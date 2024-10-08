/*
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above
   copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided
   with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


This code implements the shortest augmenting path algorithm for the
rectangular assignment problem.  This implementation is based on the
pseudocode described in pages 1685-1686 of:

    DF Crouse. On implementing 2D rectangular assignment algorithms.
    IEEE Transactions on Aerospace and Electronic Systems
    52(4):1679-1696, August 2016
    doi: 10.1109/TAES.2016.140952

Author: PM Larsen
*/

#include <cmath>
#include <vector>
#include <numeric>
#include <algorithm>
#include "hungarian_fusion.h"
#include <xtensor/xadapt.hpp>

template <typename T>
std::vector<intptr_t> argsort_iter(const std::vector<T> &v)
{
    std::vector<intptr_t> index(v.size());
    std::iota(index.begin(), index.end(), 0);
    std::sort(index.begin(), index.end(), [&v](intptr_t i, intptr_t j)
              { return v[i] < v[j]; });
    return index;
}

static intptr_t
augmenting_path(intptr_t nc, std::vector<double> &cost, std::vector<double> &u,
                std::vector<double> &v, std::vector<intptr_t> &path,
                std::vector<intptr_t> &row4col,
                std::vector<double> &shortestPathCosts, intptr_t i,
                std::vector<bool> &SR, std::vector<bool> &SC, double *p_minVal)
{
    double minVal = 0;

    // Crouse's pseudocode uses set complements to keep track of remaining
    // nodes.  Here we use a vector, as it is more efficient in C++.
    intptr_t num_remaining = nc;
    std::vector<intptr_t> remaining(nc);
    for (intptr_t it = 0; it < nc; it++)
    {
        // Filling this up in reverse order ensures that the solution of a
        // constant cost matrix is the identity matrix (c.f. #11602).
        remaining[it] = nc - it - 1;
    }

    std::fill(SR.begin(), SR.end(), false);
    std::fill(SC.begin(), SC.end(), false);
    std::fill(shortestPathCosts.begin(), shortestPathCosts.end(), INFINITY);

    // find shortest augmenting path
    intptr_t sink = -1;
    while (sink == -1)
    {

        intptr_t index = -1;
        double lowest = INFINITY;
        SR[i] = true;

        for (intptr_t it = 0; it < num_remaining; it++)
        {
            intptr_t j = remaining[it];

            double r = minVal + cost[i * nc + j] - u[i] - v[j];
            if (r < shortestPathCosts[j])
            {
                path[j] = i;
                shortestPathCosts[j] = r;
            }

            // When multiple nodes have the minimum cost, we select one which
            // gives us a new sink node. This is particularly important for
            // integer cost matrices with small co-efficients.
            if (shortestPathCosts[j] < lowest ||
                (shortestPathCosts[j] == lowest && row4col[j] == -1))
            {
                lowest = shortestPathCosts[j];
                index = it;
            }
        }

        minVal = lowest;
        intptr_t j = remaining[index];
        if (minVal == INFINITY)
        { // infeasible cost matrix
            return -1;
        }

        if (row4col[j] == -1)
        {
            sink = j;
        }
        else
        {
            i = row4col[j];
        }

        SC[j] = true;
        remaining[index] = remaining[--num_remaining];
        remaining.resize(num_remaining);
    }

    *p_minVal = minVal;
    return sink;
}

static int
solve(intptr_t nr, intptr_t nc, double *input_cost, bool maximize,
      int64_t *a, int64_t *b)
{
    // handle trivial inputs
    if (nr == 0 || nc == 0)
    {
        return 0;
    }

    std::vector<double> cost(input_cost, input_cost + nr * nc);

    // tall rectangular cost matrix must be transposed
    bool transpose = nc < nr;
    if (transpose)
    {
        for (intptr_t i = 0; i < nr; i++)
        {
            for (intptr_t j = 0; j < nc; j++)
            {
                cost[j * nr + i] = input_cost[i * nc + j];
            }
        }

        std::swap(nr, nc);
    }

    // negate cost matrix for maximization
    if (maximize)
    {
        for (intptr_t i = 0; i < nr * nc; i++)
        {
            cost[i] = -cost[i];
        }
    }

    // build a non-negative cost matrix
    double minval = *std::min_element(cost.begin(), cost.end());
    for (intptr_t i = 0; i < nr * nc; i++)
    {
        auto v = cost[i] - minval;
        cost[i] = v;

        // test for NaN and -inf entries
        if (v != v || v == -INFINITY)
        {
            return RECTANGULAR_LSAP_INVALID;
        }
    }

    // initialize variables
    std::vector<double> u(nr, 0);
    std::vector<double> v(nc, 0);
    std::vector<double> shortestPathCosts(nc);
    std::vector<intptr_t> path(nc, -1);
    std::vector<intptr_t> col4row(nr, -1);
    std::vector<intptr_t> row4col(nc, -1);
    std::vector<bool> SR(nr);
    std::vector<bool> SC(nc);

    // iteratively build the solution
    for (intptr_t curRow = 0; curRow < nr; curRow++)
    {

        double minVal;
        intptr_t sink = augmenting_path(nc, cost, u, v, path, row4col,
                                        shortestPathCosts, curRow, SR, SC, &minVal);
        if (sink < 0)
        {
            return RECTANGULAR_LSAP_INFEASIBLE;
        }

        // update dual variables
        u[curRow] += minVal;
        for (intptr_t i = 0; i < nr; i++)
        {
            if (SR[i] && i != curRow)
            {
                u[i] += minVal - shortestPathCosts[col4row[i]];
            }
        }

        for (intptr_t j = 0; j < nc; j++)
        {
            if (SC[j])
            {
                v[j] -= minVal - shortestPathCosts[j];
            }
        }

        // augment previous solution
        intptr_t j = sink;
        while (1)
        {
            intptr_t i = path[j];
            row4col[j] = i;
            std::swap(col4row[i], j);
            if (i == curRow)
            {
                break;
            }
        }
    }

    if (transpose)
    {
        intptr_t i = 0;
        for (auto v : argsort_iter(col4row))
        {
            a[i] = col4row[v];
            b[i] = v;
            i++;
        }
    }
    else
    {
        for (intptr_t i = 0; i < nr; i++)
        {
            a[i] = i;
            b[i] = col4row[i];
        }
    }

    return 0;
}

#ifdef __cplusplus
extern "C"
{
#endif

    int
    solve_rectangular_linear_sum_assignment_fusion(intptr_t nr, intptr_t nc,
                                            double *input_cost, bool maximize,
                                            int64_t *a, int64_t *b)
    {
        return solve(nr, nc, input_cost, maximize, a, b);
    }

#ifdef __cplusplus
}
#endif

xt::xarray<int> hungarian_alg(xt::xarray<float> &input)
{
    long int rows = input.shape(0);
    long int cols = input.shape(1);

    double input_cost[input.shape(0) * input.shape(1)];
    for (int i = 0; i < input.shape(0); i++)
        for (int j = 0; j < input.shape(1); j++)
        {
            input_cost[i * input.shape(1) + j] = double(input(i, j));
        }
    bool maximize = false;
    int min_dim = input.shape(0) < input.shape(1) ? input.shape(0) : input.shape(1);
    int64_t *outa = new int64_t[min_dim];
    int64_t *outb = new int64_t[min_dim];

    int temp = solve_rectangular_linear_sum_assignment_fusion(rows, cols, input_cost, maximize, outa, outb);
    std::vector<int> out_temp;
    for (int i = 0; i < min_dim; i++)
    {
        out_temp.push_back(outa[i]);
        out_temp.push_back(outb[i]);
    }
    delete []outa;
    delete []outb;
    xt::xarray<int> output = xt::adapt(out_temp, {min_dim, 2});
    return output;
};