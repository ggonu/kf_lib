#include <iostream>
#include "kf.hpp"

#define PI 3.14159265

KF::KF() {}

KF::~KF() {}

void KF::Init(Eigen::VectorXd& x_in, Eigen::MatrixXd& P_in,
              Eigen::MatrixXd& F_in, Eigen::MatrixXd& H_in,
              Eigen::MatrixXd& R_in, Eigen::MatrixXd& Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KF::Predict() {
  // x = F*x + B*u
  x_ = x_ * F_;

  // P = F*P*Ft + Q
  Eigen::MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KF::Update(const Eigen::VectorXd& z) {
  

}

