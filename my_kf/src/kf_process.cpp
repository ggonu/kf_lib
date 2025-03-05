#include "kf_process.hpp"
#include "kf_funcs.hpp"
#include "ekf.hpp"

#include "Eigen/Dense"

#include <iostream>

KFProcess::KFProcess() {
  is_init_ = false;

  prev_t_ = 0;

  // Init matrices
  R_lidar_ = Eigen::MatrixXd(2, 2);
  R_radar_ = Eigen::MatrixXd(3, 3);
  H_lidar_ = Eigen::MatrixXd(2, 4);
  H_jacob_ = Eigen::MatrixXd(3, 4);

  // Measurement covariance matrix
  // Examples
  // LiDAR & Radar
  /**
   * R_lidar_ << 0.0225, 0.0,
   *             0.0, 0.0225;
   *
   * R_radar_ << 0.09, 0.0, 0.0,
   *             0.0, 0.0009, 0.0,
   *             0.0, 0.0, 0.09;
   */


  // Measurement matrix
  
  
}

KFProcess::~KFProcess() {}

void KFProcess::ProcessMeasurement() {}