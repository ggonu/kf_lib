#include <iostream>

#include "kf.hpp"


#define PI 3.14159265


KF::KF() {}

KF::~KF() {}

void KF::Init(Eigen::VectorXd& x_in, Eigen::MatrixXd& P_in,
              Eigen::MatrixXd& F_in, Eigen::VectorXd& u_in,
              Eigen::MatrixXd& B_in, Eigen::MatrixXd& H_in,
              Eigen::MatrixXd& R_in, Eigen::MatrixXd& Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  u_ = u_in;
  B_ = B_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KF::Predict() {
  Eigen::MatrixXd Ft = F_.transpose();
  // x = F*x + B*u
  x_ = x_ * F_ + B_ * u_;

  // P = F*P*Ft + Q
  P_ = F_ * P_ * Ft + Q_;
}

void KF::Update(const Eigen::VectorXd& z) {
  Eigen::MatrixXd Ht = H_.transpose();
  Eigen::MatrixXd PHt = P_ * Ht;

  Eigen::VectorXd y = z - H_ * x_;
  Eigen::MatrixXd S = H_ * PHt + R_;
  Eigen::MatrixXd K = PHt * S.inverse();

  x_ = x_ + K * y;
  long x_size = x_.size();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

