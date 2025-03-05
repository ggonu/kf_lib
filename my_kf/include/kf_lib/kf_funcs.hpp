#ifndef KF_FUNCS_HPP_
#define KF_FUNCS_HPP_

#include "Eigen/Dense"
#include <vector>

class Funcs {
public:
  // Constructor.
  Funcs();

  // Destructor.
  virtual ~Funcs();

  /**
   * Calculate the RMSE (Root Mean Square Error):
   * Ensure the input vectors are not empty and have same size
   * @param estimations A vector of state estimations
   * @param ground_truth A vector of true state values
   * @return RMSE as a VectorXd
   */
  Eigen::VectorXd
  CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                const std::vector<Eigen::VectorXd> &ground_truth);
};

#endif // KF_FUNCS_HPP_