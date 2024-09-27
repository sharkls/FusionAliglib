#ifndef HUNGARIAN_ALGORITHM_H
#define HUNGARIAN_ALGORITHM_H

#include <vector>

class HungarianAlgorithm {
public:
    static std::vector<int> solve(const std::vector<std::vector<double>> &costMatrix);
};

#endif // HUNGARIAN_ALGORITHM_H