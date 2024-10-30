#ifndef EKF_FUSION_HPP_
#define EKF_FUSION_HPP_

#include <vector>
#include "Eigen/Dense"

#include "kf.hpp"
#include "ekf.hpp"
#include "kf_funcs.hpp"

class EKFFusion {
  public:
    // Constructor.
    EKFFusion();
    
    // Destructor.
    virtual ~EKFFusion();

    /**
     * Process the Kalman Filter or Extended Kalman Filter
     */
    void ProcessMeasurement();

  private:
    bool is_init_;

    // Must add matricex : Measurements, Noises, etc,.
};


    
#endif // EKF_FUSION_HPP_