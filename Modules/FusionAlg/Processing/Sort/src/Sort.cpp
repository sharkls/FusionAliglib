#include "Sort.h"

Sort::Sort(int max_age, int min_hits, bool use_acceleration) 
    : max_age_(max_age), min_hits_(min_hits), frame_count_(0), use_acceleration_(use_acceleration) {}

std::vector<Eigen::VectorXd> Sort::update(const std::vector<Eigen::VectorXd> &detections) {
    frame_count_++;
    std::vector<Eigen::VectorXd> ret;

    // Predict new locations of existing trackers
    for (auto &tracker : trackers_) {
        tracker.predict();
    }

    // Associate detections to trackers
    auto matches = associateDetectionsToTrackers(detections);

    // Update matched trackers with assigned detections
    for (const auto &match : matches) {
        trackers_[match.second].update(detections[match.first]);
    }

    // Create and initialize new trackers for unmatched detections
    for (size_t i = 0; i < detections.size(); ++i) {
        if (std::find_if(matches.begin(), matches.end(), [i](const std::pair<int, int> &match) { return match.first == i; }) == matches.end()) {
            KalmanFilter tracker;
            tracker.init(detections[i], use_acceleration_);
            trackers_.push_back(tracker);
        }
    }

    // Collect results
    for (auto &tracker : trackers_) {
        if (tracker.getState()(6) >= min_hits_ || frame_count_ <= min_hits_) {
            ret.push_back(tracker.getState());
        }
    }

    // Remove dead trackers
    trackers_.erase(std::remove_if(trackers_.begin(), trackers_.end(), [this](const KalmanFilter &tracker) {
        return tracker.getState()(6) > max_age_;
    }), trackers_.end());

    return ret;
}

std::vector<std::pair<int, int>> Sort::associateDetectionsToTrackers(const std::vector<Eigen::VectorXd> &detections) {
    // Compute IoU matrix
    std::vector<std::vector<double>> iouMatrix(detections.size(), std::vector<double>(trackers_.size(), 0));
    for (size_t i = 0; i < detections.size(); ++i) {
        for (size_t j = 0; j < trackers_.size(); ++j) {
            iouMatrix[i][j] = iou(detections[i], trackers_[j].getState());
        }
    }

    // Solve assignment problem using Hungarian algorithm
    auto assignment = HungarianAlgorithm::solve(iouMatrix);

    // Convert assignment to pairs of matched indices
    std::vector<std::pair<int, int>> matches;
    for (size_t i = 0; i < assignment.size(); ++i) {
        if (assignment[i] != -1) {
            matches.emplace_back(i, assignment[i]);
        }
    }

    return matches;
}

double Sort::iou(const Eigen::VectorXd &bb_test, const Eigen::VectorXd &bb_gt) {
    double xx1 = std::max(bb_test(0), bb_gt(0));
    double yy1 = std::max(bb_test(1), bb_gt(1));
    double xx2 = std::min(bb_test(2), bb_gt(2));
    double yy2 = std::min(bb_test(3), bb_gt(3));
    double w = std::max(0.0, xx2 - xx1);
    double h = std::max(0.0, yy2 - yy1);
    double wh = w * h;
    double o = wh / ((bb_test(2) - bb_test(0)) * (bb_test(3) - bb_test(1)) + (bb_gt(2) - bb_gt(0)) * (bb_gt(3) - bb_gt(1)) - wh);
    return o;
}