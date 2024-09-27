#ifndef SORT_H
#define SORT_H

#include "KalmanFilter.h"
#include "HungarianAlgorithm.h"
#include <vector>
#include <Eigen/Dense>

class Sort {
public:
    Sort(int max_age = 1, int min_hits = 3, bool use_acceleration = false);
    std::vector<Eigen::VectorXd> update(const std::vector<Eigen::VectorXd> &detections);

private:
    int max_age_;
    int min_hits_;
    int frame_count_;
    bool use_acceleration_;
    std::vector<KalmanFilter> trackers_;

    std::vector<std::pair<int, int>> associateDetectionsToTrackers(const std::vector<Eigen::VectorXd> &detections);
    double iou(const Eigen::VectorXd &bb_test, const Eigen::VectorXd &bb_gt);
};

#endif // SORT_H