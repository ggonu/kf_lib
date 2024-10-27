/* --------------------------- *
 * Kalman Filter
 * - Basic Structure
 *    - variables & parameters
 *      - scalars
 *      - vectors
 *      - matrices
 * 
 *    - functions
 *      - estimation
 *      - update
 *  -------------------------- */

#ifndef KF_HPP_
#define KF_HPP_

#include <iostream>
#include <Eigen/Dense>


class KF {
  public:
    // double dt;
    Eigen::VectorXd x_; // State Vector 
    Eigen::MatrixXd P_; // State Cov. Matrix
    Eigen::MatrixXd F_; // State trans. Matrix
    Eigen::MatrixXd Q_; // Cov. Matrix
    Eigen::MatrixXd H_; // Measurement Matrix
    Eigen::MatrixXd R_; // Measurement Cov. Matrix
    

    // Constructor
    KF();
    // Destructor
    ~KF();

    /**
     * Check the Matrix
     */
    void ShowMatrix(Eigen::MatrixXd& matrix) {
      std::cout << "Matrix:\n" << matrix << std::endl;
    }

    /**
     * Check the Vector
     */
    void ShowVector(Eigen::VectorXd& vector) {
      std::cout << "Vector:\n" << vector << std::endl;
    }

    /**
     * Check Vector and Matrices of the Kalman Filter
     */
    void ShowParams() {
      std::cout << "State Vector:\n" << x_ << std::endl;
      std::cout << "State covariance Matrix:\n" << P_ << std::endl;
      std::cout << "State transition Matrix:\n" << F_ << std::endl;
      std::cout << "Covariance Matrix:\n" << Q_ << std::endl;
      std::cout << "Measurement Matrix:\n" << H_ << std::endl;
      std::cout << "Measurement covariance Matrix:\n" << R_ << std::endl;
    }

    /**
     * Initialize the Kalman Filter
     * @param x_in Initial state 
     * @param P_in Initial state covariance
     * @param F_in Transition matrix
     * @param H_in Measurement matrix
     * @param R_in Measurement covariance matrix
     * @param Q_in Process covariance matrix
     */
    void Init(Eigen::VectorXd& x_in, Eigen::MatrixXd& P_in,
              Eigen::MatrixXd& F_in, Eigen::MatrixXd& H_in,
              Eigen::MatrixXd& R_in, Eigen::MatrixXd& Q_in);

    /**
     * Prediction Step of Kalman Filter
     * @param delta_T Time step
     */
    void Predict();

    /**
     * Update Step of Kalman Filter
     * Update the state by using Kalman Filter Eq.
     * @param z Measurement at time k+1
     */
    void Update(const Eigen::VectorXd& z);
    


  private:
};


#endif  // KF_HPP_