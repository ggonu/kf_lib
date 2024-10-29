#include <iostream>

#include "ekf.hpp"


#define PI 3.14159265
#define EPS 0.0001

EKF::EKF() {}

EKF::~EKF() {}

void EKF::Init(Eigen::VectorXd& x_in, Eigen::MatrixXd& P_in,
              Eigen::MatrixXd& F_in, Eigen::VectorXd& u_in,
              Eigen::MatrixXd& B_in, Eigen::MatrixXd& H_in,
              Eigen::MatrixXd& R_in, Eigen::MatrixXd& Q_in) {
              // std::function<Eigen::VectorXd(const Eigen::VectorXd&)> f_in,
              // std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h_in) {
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

Eigen::MatrixXd EKF::CalculateJacobian(const Eigen::VectorXd& x) {
  Eigen::MatrixXd Hj(H_dim_, x_dim_);
  Hj.setZero();

  // Perturbation vector
  Eigen::VectorXd x_p = x;

  // Conpute the Jacobian
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

void EKF::Predict() {
  // x_ = f(x_, u_) + w_;
  x_ = f_(x_);

  // P = F*P*Ft + Q
  Eigen::MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void EKF::Update(const Eigen::VectorXd& z) {
  Eigen::MatrixXd H_jacobi = EKF::CalculateJacobian(x_); // Jacobian matrix of h_()
  Eigen::MatrixXd Ht = H_.transpose();
  Eigen::MatrixXd PHt = P_ * Ht;

  Eigen::VectorXd y = z - h_(x_);
  Eigen::MatrixXd S = H_jacobi * PHt + R_;
  Eigen::MatrixXd K = PHt * S.inverse();

  x_ = x_ + K * y;
  // long x_size = x_.size();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_dim_, x_dim_);
  P_ = (I - K * H_jacobi) * P_;
}

