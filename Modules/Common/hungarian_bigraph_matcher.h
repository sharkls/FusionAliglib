#pragma once

#include "hungarian_optimizer.h"
#include "secure_matrix.h"
#include "xtensor/xarray.hpp"
#include <algorithm>
#include <functional>
#include <iostream>
#include <map>
#include <utility>
#include <vector>
#include <queue>


template <typename T>
void ConnectedComponentAnalysis(const std::vector<std::vector<int>>& graph,
                                std::vector<std::vector<int>>* components) {
  if (components == nullptr) {
    // AERROR << "components is not available";
    return;
  }
  int num_item = static_cast<int>(graph.size());
  std::vector<int> visited;
  visited.resize(num_item, 0);
  std::queue<int> que;
  std::vector<int> component;
  component.reserve(num_item);
  components->clear();

  for (int index = 0; index < num_item; ++index) {
    if (visited[index]) {
      continue;
    }
    component.push_back(index);
    que.push(index);
    visited[index] = 1;
    while (!que.empty()) {
      int current_id = que.front();
      que.pop();
      for (size_t sub_index = 0; sub_index < graph[current_id].size();
           ++sub_index) {
        int neighbor_id = graph[current_id][sub_index];
        if (visited[neighbor_id] == 0) {
          component.push_back(neighbor_id);
          que.push(neighbor_id);
          visited[neighbor_id] = 1;
        }
      }
    }
    components->push_back(component);
    component.clear();
  }
}

template <typename T> 
class GatedHungarianMatcher {
public:
  enum class OptimizeFlag { OPTMAX, OPTMIN };

  explicit GatedHungarianMatcher(int max_matching_size = 500) {
    global_costs_.Reserve(max_matching_size, max_matching_size);
    optimizer_.costs()->Reserve(max_matching_size, max_matching_size);
  }
  ~GatedHungarianMatcher() {}

  const SecureMat<T> &global_costs() const { return global_costs_; }
  SecureMat<T> *mutable_global_costs() { return &global_costs_; }

  void Match(T cost_thresh, OptimizeFlag opt_flag,
             std::vector<std::pair<size_t, size_t>> *assignments);

  void Match(std::vector<std::pair<size_t, size_t>> *assignments);
  // void ConnectedComponentAnalysis(const std::vector<std::vector<int>>& graph,
  //                               std::vector<std::vector<int>>* components);
  
private:
  /* Step 1:
   * a. get number of rows & cols
   * b. determine function of comparison */
  void MatchInit();
  /* Step 2:
   * to acclerate matching process, split input cost graph into several
   * small sub-parts. */
  void ComputeConnectedComponents(
      std::vector<std::vector<size_t>> *row_components,
      std::vector<std::vector<size_t>> *col_components) const;

  /* Step 3:
   * optimize single connected component, which is part of the global one */
  void OptimizeConnectedComponent(const std::vector<size_t> &row_component,
                                  const std::vector<size_t> &col_component);

  /* Step 4:
   * generate the set of unassigned row or col index. */
  void GenerateUnassignedData(std::vector<size_t> *unassigned_rows,
                              std::vector<size_t> *unassigned_cols) const;

  void UpdateGatingLocalCostsMat(const std::vector<size_t> &row_component,
                                 const std::vector<size_t> &col_component);

  void
  OptimizeAdapter(std::vector<std::pair<size_t, size_t>> *local_assignments);

  /* Hungarian optimizer */
  HungarianOptimizer<T> optimizer_;

  /* global costs matrix */
  SecureMat<T> global_costs_;

  /* input data */
  T cost_thresh_ = 10000.0f;
  T bound_value_ = 10000.0f;
  OptimizeFlag opt_flag_ = OptimizeFlag::OPTMIN;

  /* output data */
  mutable std::vector<std::pair<size_t, size_t>> *assignments_ptr_ = nullptr;

  /* size of component */
  size_t rows_num_ = 0;
  size_t cols_num_ = 0;

  /* the rhs is always better than lhs */
  std::function<bool(T, T)> compare_fun_;
  std::function<bool(T)> is_valid_cost_;
}; // class GatedHungarianMatcher

template <typename T>
void GatedHungarianMatcher<T>::Match(
    std::vector<std::pair<size_t, size_t>> *assignments) {

  /* initialize matcher */
  assignments_ptr_ = assignments;
  MatchInit();

  /* compute components */
  std::vector<std::vector<size_t>> row_components;
  std::vector<std::vector<size_t>> col_components;
  this->ComputeConnectedComponents(&row_components, &col_components);

  /* compute assignments */
  assignments_ptr_->clear();
  assignments_ptr_->reserve(std::max(rows_num_, cols_num_));
  for (size_t i = 0; i < row_components.size(); ++i) {
    this->OptimizeConnectedComponent(row_components[i], col_components[i]);
  }

  // this->GenerateUnassignedData(unassigned_rows, unassigned_cols);
}

template <typename T> void GatedHungarianMatcher<T>::MatchInit() {
  /* get number of rows & cols */
  rows_num_ = global_costs_.height();
  cols_num_ = (rows_num_ == 0) ? 0 : global_costs_.width();

  /* determine function of comparison */
  static std::map<OptimizeFlag, std::function<bool(T, T)>> compare_fun_map = {
      {OptimizeFlag::OPTMAX, std::less<T>()},
      {OptimizeFlag::OPTMIN, std::greater<T>()},
  };
  auto find_ret = compare_fun_map.find(opt_flag_);

  compare_fun_ = find_ret->second;
  is_valid_cost_ = std::bind1st(compare_fun_, cost_thresh_);
}

template <typename T>
void GatedHungarianMatcher<T>::ComputeConnectedComponents(
    std::vector<std::vector<size_t>> *row_components,
    std::vector<std::vector<size_t>> *col_components) const {

  std::vector<std::vector<int>> nb_graph;
  nb_graph.resize(rows_num_ + cols_num_);
  for (size_t i = 0; i < rows_num_; ++i) {
    for (size_t j = 0; j < cols_num_; ++j) {
      if (is_valid_cost_(global_costs_(i, j))) {
        nb_graph[i].push_back(static_cast<int>(rows_num_) + j);
        nb_graph[j + rows_num_].push_back(i);
      }
    }
  }

  std::vector<std::vector<int>> components;
  // ConnectedComponentAnalysis(nb_graph, &components);
  ConnectedComponentAnalysis<float>(nb_graph, &components);
 
  row_components->clear();
  row_components->resize(components.size());
  col_components->clear();
  col_components->resize(components.size());
  for (size_t i = 0; i < components.size(); ++i) {
    for (size_t j = 0; j < components[i].size(); ++j) {
      int id = components[i][j];
      if (id < static_cast<int>(rows_num_)) {
        row_components->at(i).push_back(id);
      } else {
        id -= static_cast<int>(rows_num_);
        col_components->at(i).push_back(id);
      }
    }
  }
}

template <typename T>
void GatedHungarianMatcher<T>::OptimizeConnectedComponent(
    const std::vector<size_t> &row_component,
    const std::vector<size_t> &col_component) {
  size_t local_rows_num = row_component.size();
  size_t local_cols_num = col_component.size();

  /* simple case 1: no possible matches */
  if (!local_rows_num || !local_cols_num) {
    return;
  }
  /* simple case 2: 1v1 pair with no ambiguousness */
  if (local_rows_num == 1 && local_cols_num == 1) {
    size_t idx_r = row_component[0];
    size_t idx_c = col_component[0];
    if (is_valid_cost_(global_costs_(idx_r, idx_c))) {
      assignments_ptr_->push_back(std::make_pair(idx_r, idx_c));
    }
    return;
  }

  /* update local cost matrix */
  UpdateGatingLocalCostsMat(row_component, col_component);

  /* get local assignments */
  std::vector<std::pair<size_t, size_t>> local_assignments;
  OptimizeAdapter(&local_assignments);

  /* parse local assginments into global ones */
  for (size_t i = 0; i < local_assignments.size(); ++i) {
    auto local_assignment = local_assignments[i];
    size_t global_row_idx = row_component[local_assignment.first];
    size_t global_col_idx = col_component[local_assignment.second];
    if (!is_valid_cost_(global_costs_(global_row_idx, global_col_idx))) {
      continue;
    }
    assignments_ptr_->push_back(std::make_pair(global_row_idx, global_col_idx));
  }
}

template <typename T>
void GatedHungarianMatcher<T>::GenerateUnassignedData(
    std::vector<size_t> *unassigned_rows,
    std::vector<size_t> *unassigned_cols) const {

  const auto assignments = *assignments_ptr_;
  unassigned_rows->clear(), unassigned_rows->reserve(rows_num_);
  unassigned_cols->clear(), unassigned_cols->reserve(cols_num_);
  std::vector<bool> row_assignment_flags(rows_num_, false);
  std::vector<bool> col_assignment_flags(cols_num_, false);
  for (const auto &assignment : assignments) {
    row_assignment_flags[assignment.first] = true;
    col_assignment_flags[assignment.second] = true;
  }
  for (size_t i = 0; i < row_assignment_flags.size(); ++i) {
    if (!row_assignment_flags[i]) {
      unassigned_rows->push_back(i);
    }
  }
  for (size_t i = 0; i < col_assignment_flags.size(); ++i) {
    if (!col_assignment_flags[i]) {
      unassigned_cols->push_back(i);
    }
  }
}

template <typename T>
void GatedHungarianMatcher<T>::UpdateGatingLocalCostsMat(
    const std::vector<size_t> &row_component,
    const std::vector<size_t> &col_component) {
  /* set the invalid cost to bound value */
  SecureMat<T> *local_costs = optimizer_.costs();
  local_costs->Resize(row_component.size(), col_component.size());
  for (size_t i = 0; i < row_component.size(); ++i) {
    for (size_t j = 0; j < col_component.size(); ++j) {
      T &current_cost = global_costs_(row_component[i], col_component[j]);
      if (is_valid_cost_(current_cost)) {
        (*local_costs)(i, j) = current_cost;
      } else {
        (*local_costs)(i, j) = bound_value_;
      }
    }
  }
}

template <typename T>
void GatedHungarianMatcher<T>::OptimizeAdapter(
    std::vector<std::pair<size_t, size_t>> *local_assignments) {
  // CHECK_NOTNULL(local_assignments);
  if (opt_flag_ == OptimizeFlag::OPTMAX) {
    optimizer_.Maximize(local_assignments);
  } else {
    optimizer_.Minimize(local_assignments);
  }
}

// 输入为正即可, 默认保持与scipy一致, 然后求最小代价匹配.如果想要与KM保持一样,请将opt_flag_改为OPTMAX;
template<typename T>
xt::xarray<int> run_hungarian_match(xt::xarray<T> & input){
    GatedHungarianMatcher<float>* optimizer_ = new GatedHungarianMatcher<float>();
    SecureMat<float>* global_costs = optimizer_->mutable_global_costs();
    std::vector<std::pair<size_t, size_t>> assignments;

    global_costs->Resize(input.shape(0), input.shape(1));
    for (int i = 0; i < input.shape(0); ++i){
        for (int j = 0; j < input.shape(1); ++j) {
            (*global_costs)(i, j) =  input(i, j);
        }
    }
    optimizer_->Match(&assignments);
    delete optimizer_;
    xt::xarray<int> output = xt::zeros<int>({int(assignments.size()), 2});
    for (int i = 0; i < assignments.size(); ++i) {
        output(i, 0) = assignments[i].first;
        output(i, 1) = assignments[i].second;
    }
    return output;
}