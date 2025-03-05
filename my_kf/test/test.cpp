#include <iostream>
#include "kf.hpp"
#include "ekf.hpp"


int main() {
  // Initialize state vector
  Eigen::VectorXd x(4);
  x << 0, 0, 0, 0;

  // Initialize state covariance matrix
  Eigen::MatrixXd P(4, 4);
  P.setIdentity();

  // Initialize state transition matrix
  Eigen::MatrixXd F(4, 4);
  F.setIdentity();

  // Initialize control input vector
  Eigen::VectorXd u(4);
  u.setZero();

  // Initialize control input matrix
  Eigen::MatrixXd B(4, 4);
  B.setIdentity();

  // Initialize measurement matrix for LiDAR
  Eigen::MatrixXd H_lidar(2, 4);
  H_lidar << 1, 0, 0, 0,
             0, 1, 0, 0;

  // Initialize measurement covariance matrix for LiDARs
  Eigen::MatrixXd R_lidar(2, 2);
  R_lidar.setIdentity();

  // Initialize process covariance matrix
  Eigen::MatrixXd Q(4, 4);
  Q.setIdentity();

  // Initialize the Kalman Filter
  KF kf;
  kf.Init(x, P, F, u, B, H_lidar, R_lidar, Q);

  // Simulate a measurement from LiDAR
  Eigen::VectorXd z_lidar(2);
  z_lidar << 1, 1;

  // Predict and update state using KF
  kf.Predict();
  kf.Update(z_lidar);

  std::cout << "State after KF update: " << std::endl;
  kf.ShowParams();

  // Initialize measurement matrix for radar
  auto H_radar = [](const Eigen::VectorXd& x) -> Eigen::VectorXd {
    Eigen::VectorXd z(3);
    float px = x(0);
    float py = x(1);
    float vx = x(2);
    float vy = x(3);
    float rho = sqrt(px * px + py * py);
    float phi = atan2(py, px);
    float rho_dot = (px * vx + py * vy) / rho;
    z << rho, phi, rho_dot;
    return z;
  };

  // Initialize measurement covariance matrix for radar
  Eigen::MatrixXd R_radar(3, 3);
  R_radar.setIdentity();

  EKF ekf;
  ekf.Init(x, P, F, u, B, R_radar, Q, H_radar);

  // Simulate a measurement from Radar
  Eigen::VectorXd z_radar(3);
  z_radar << 1, 0.5, 0.1;

  // Predict and update state using EKF
  ekf.Predict();
  ekf.Update(z_radar);

  std::cout << "State after EKF update: " << std::endl;
  ekf.ShowParams();

  return 0;
}