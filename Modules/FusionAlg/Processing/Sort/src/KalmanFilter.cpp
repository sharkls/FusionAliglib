#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {
    // Initialize matrices
    x_ = Eigen::VectorXd(6); // [x, y, z, vx, vy, vz] for constant velocity model
    P_ = Eigen::MatrixXd::Identity(6, 6);
    F_ = Eigen::MatrixXd::Identity(6, 6);
    Q_ = Eigen::MatrixXd::Identity(6, 6);
    H_ = Eigen::MatrixXd::Zero(3, 6); // [x, y, z]
    R_ = Eigen::MatrixXd::Identity(3, 3);

    // Set measurement matrix
    H_.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3);

    // Adjust process noise covariance matrix
    Q_.diagonal() << 1, 1, 1, 0.1, 0.1, 0.1;

    // Adjust measurement noise covariance matrix
    R_.diagonal() << 1, 1, 1;
}

void KalmanFilter::init(const Eigen::VectorXd &x_in, bool use_acceleration) {
    x_ = x_in;
    use_acceleration_ = use_acceleration;

    if (use_acceleration_) {
        // Extend state vector and matrices for acceleration model
        x_.conservativeResize(9); // [x, y, z, vx, vy, vz, ax, ay, az]
        P_.conservativeResize(9, 9);
        F_.conservativeResize(9, 9);
        Q_.conservativeResize(9, 9);
        H_.conservativeResize(3, 9);

        P_.bottomRows(3).setZero();
        P_.rightCols(3).setZero();
        P_.diagonal().tail(3).setConstant(1000);

        F_.bottomRows(3).setZero();
        F_.rightCols(3).setZero();
        F_.diagonal().tail(3).setConstant(1);

        Q_.bottomRows(3).setZero();
        Q_.rightCols(3).setZero();
        Q_.diagonal().tail(3).setConstant(0.1);

        H_.rightCols(3).setZero();
    }
}

void KalmanFilter::predict() {
    if (use_acceleration_) {
        // Update state transition matrix for acceleration model
        F_.block<3, 3>(0, 3) = Eigen::MatrixXd::Identity(3, 3);
        F_.block<3, 3>(3, 6) = Eigen::MatrixXd::Identity(3, 3);
    } else {
        // Update state transition matrix for constant velocity model
        F_.block<3, 3>(0, 3) = Eigen::MatrixXd::Identity(3, 3);
    }

    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const Eigen::VectorXd &z) {
    Eigen::VectorXd y = z - H_ * x_;
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (Eigen::MatrixXd::Identity(x_.size(), x_.size()) - K * H_) * P_;
}

Eigen::VectorXd KalmanFilter::getState() {
    return x_;
}