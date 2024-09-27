#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter();
    void init(const Eigen::VectorXd &x_in, bool use_acceleration = false);
    void predict();
    void update(const Eigen::VectorXd &z);

    Eigen::VectorXd getState();

private:
    Eigen::VectorXd x_; // state vector
    Eigen::MatrixXd P_; // state covariance matrix
    Eigen::MatrixXd F_; // state transition matrix
    Eigen::MatrixXd Q_; // process covariance matrix
    Eigen::MatrixXd H_; // measurement matrix
    Eigen::MatrixXd R_; // measurement covariance matrix
    bool use_acceleration_; // flag to use acceleration model
};

#endif // KALMAN_FILTER_H