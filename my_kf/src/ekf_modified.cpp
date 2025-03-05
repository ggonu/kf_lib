#include "ekf_modified.hpp"
#include <iostream>

#define PI 3.14159265
#define EPS 1e-4

EKFModi::EKFModi() {}

EKFModi::~EKFModi() {}

// Initialize EKF with reliability-based fusion
void EKFModi::Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in,
                   Eigen::MatrixXd &F_in, Eigen::VectorXd &u_in,
                   Eigen::MatrixXd &B_in, Eigen::MatrixXd &H_in,
                   Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  u_ = u_in;
  B_ = B_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;

  x_dim_ = x_.size();
  H_dim_ = h_(x_).size();
}

// State transition prediction step (update based on velocity, angle, and time
// delta)
void EKFModi::Predict(double delta_t) {
  // Update state transition matrix F based on the time interval and current
  // state
  F_(0, 2) = delta_t * cos(x_(3)); // x-direction
  F_(1, 2) = delta_t * sin(x_(3)); // y-direction

  x_ = F_ * x_ + B_ * u_; // Predict state

  // P = F*P*Ft + Q
  Eigen::MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

Eigen::MatrixXd EKFModi::CalculateJacobian(const Eigen::VectorXd &x) {
  Eigen::MatrixXd Hj(H_dim_, x_dim_);
  Hj.setZero();

  // Perturbation vector
  Eigen::VectorXd x_p = x;

  // Compute the Jacobian
  for (int i = 0; i < x_dim_; ++i) {
    float val_ori = x(i); // Store value

    x_p(i) = val_ori + EPS;
    Eigen::VectorXd h_p = h_(x_p);

    x_p(i) = val_ori - EPS;
    Eigen::VectorXd h_n = h_(x_p);

    x_p(i) = val_ori; // Reset value

    // Calculate derivative
    Eigen::VectorXd dh = (h_p - h_n) / (2 * EPS);
    Hj.col(i) = dh;
  }

  return Hj;
}

// Reliability-based measurement update
void EKFModi::Update(const Eigen::VectorXd &z) {
  // Calculate the Jacobian for non-linear measurement
  Eigen::MatrixXd H_jacobi = CalculateJacobian(x_);
  Eigen::MatrixXd Ht = H_jacobi.transpose();
  Eigen::MatrixXd PHt = P_ * Ht;

  // Calculate reliability-adjusted residual
  Eigen::VectorXd y = z - h_(x_); // Measurement error
  ApplyReliabilityFunction(y);    // Adjust based on sensor distance reliability

  Eigen::MatrixXd S = H_jacobi * PHt + R_;
  Eigen::MatrixXd K = PHt * S.inverse(); // Kalman gain with reliability

  // Update state with Kalman gain and reliability-adjusted measurement
  x_ = x_ + K * y;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_dim_, x_dim_);
  P_ = (I - K * H_jacobi) * P_;
}

// Apply reliability function to adjust measurement residual based on distance
void EKFModi::ApplyReliabilityFunction(Eigen::VectorXd &residual) {
  double distance = residual.head<2>().norm();
  double alpha = 1.0, beta = 2.0,
         X_reli = 10.0; // Reliability tuning parameters

  // Sigmoid reliability function
  double reliability = beta / (1.0 + exp(alpha * (distance - X_reli)));
  residual *= reliability; // Scale residual by reliability
}