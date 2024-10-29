#include "kf_funcs.hpp"

Eigen::VectorXd Funcs::CalculateRMSE(const std::vector<Eigen::VectorXd>& estimations,
                                     const std::vector<Eigen::VectorXd>& ground_truth) {
  if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
    throw std::invalid_argument("Invalid estimation or ground_tryth data");
  }

  Eigen::VectorXd rmse(estimations[0].size());
  rmse.setZero();

  for (size_t i = 0; i < estimations.size(); ++i) {
    Eigen::VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  rmse /= estimations.size();

  rmse = rmse.array().sqrt();

  return rmse;
}