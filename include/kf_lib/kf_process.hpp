#ifndef KF_PROCESS_HPP_
#define KF_PROCESS_HPP_

#include <vector>
#include "Eigen/Dense"

#include "kf.hpp"
#include "ekf.hpp"
#include "kf_funcs.hpp"

class KFProcess {
  public:
    // Constructor.
    KFProcess();
    
    // Destructor.
    virtual ~KFProcess();

    /**
     * Process the Kalman Filter or Extended Kalman Filter
     */
    void ProcessMeasurement();

    /**
     *
     */
    KF kf_;
    EKF ekf_;

  private:
    bool is_init_;

    long long prev_t_;

    // Must add matricex : Measurements, Noises, etc,.
    Funcs funcs;
    Eigen::MatrixXd R_lidar_;
    Eigen::MatrixXd R_radar_;
    Eigen::MatrixXd H_lidar_;
    Eigen::MatrixXd H_jacob_;

    float noise_x_;
    float noise_y_;
};



#endif // KF_PROCESS_HPP_