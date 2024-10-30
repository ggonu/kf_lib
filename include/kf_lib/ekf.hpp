/* --------------------------- *
 * Extended Kalman Filter
 * - Basic Structure
 *    - variables & parameters
 *      - scalars: lowercase
 *      - vectors: lowercase
 *      - matrices: uppercase
 *
 *    - functions
 *      - estimation
 *      - update
 *  -------------------------- */

#ifndef EKF_HPP_
#define EKF_HPP_

#include <iostream>
#include <Eigen/Dense>


class EKF {
  public:
    // Constructor.
    EKF();
    // Destructor.
    virtual ~EKF();

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
     * @param x_ Initial state
     * @param P_ Initial state covariance
     * @param F_ Transition matrix
     * @param u_ Control input vector
     * @param B_ Control input matrix
     * @param H_ Measurement matrix
     * @param R_ Measurement covariance matrix
     * @param Q_ Process covariance matrix
     */
    void ShowParams() {
      std::cout << "State Vector:\n" << x_ << std::endl;
      std::cout << "State covariance Matrix:\n" << P_ << std::endl;
      std::cout << "State transition Matrix:\n" << F_ << std::endl;
      std::cout << "Control input Vector:\n" << u_ << std::endl;
      std::cout << "Control input Matrix:\n" << B_ << std::endl;
      std::cout << "Covariance Matrix:\n" << Q_ << std::endl;
      std::cout << "Measurement Matrix:\n" << H_ << std::endl;
      std::cout << "Measurement covariance Matrix:\n" << R_ << std::endl;
    }

    /**
     * Initialize the Extended Kalman Filter
     * @param x_in Initial state
     * @param P_in Initial state covariance
     * @param F_in Transition matrix
     * @param u_in Control input vector
     * @param B_in Control input matrix
     * @param H_in Measurement matrix
     * @param R_in Measurement covariance matrix
     * @param Q_in Process covariance matrix
     * @param f_in Non-linear state transition function
     * @param h_in Non-linear measurement function
     */
    void Init(Eigen::VectorXd& x_in, Eigen::MatrixXd& P_in,
              Eigen::MatrixXd& F_in, Eigen::VectorXd& u_in,
              Eigen::MatrixXd& B_in, Eigen::MatrixXd& H_in,
              Eigen::MatrixXd& R_in, Eigen::MatrixXd& Q_in);
              // std::function<Eigen::VectorXd(const Eigen::VectorXd&)> f_in,
              // std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h_in);

    /**
     * Calculate Jacobian of vector
     * Specially used for linearizing the Radar measurement
     * e.g. state vector x
     * @param x State vector (position and velocity components)
     * @return Jacobian Matrix for the Radar measurement model
     */
    Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x);

    /**
     * Prediction Step of Extended Kalman Filter
     */
    void Predict();

    /**
     * Update Step of Extended Kalman Filter
     * Update the state by using Extended Kalman Filter Eq.
     * @param z Measurement at time k+1
     */
    void Update(const Eigen::VectorXd& z);


  private:
    Eigen::VectorXd x_; // State Vector
    Eigen::MatrixXd P_; // State Cov. Matrix
    Eigen::MatrixXd F_; // State trans. Matrix
    Eigen::VectorXd u_; // Control input Vector
    Eigen::MatrixXd B_; // Control input Matrix
    Eigen::MatrixXd Q_; // Cov. Matrix
    Eigen::MatrixXd H_; // Measurement Matrix
    Eigen::MatrixXd R_; // Measurement Cov. Matrix

    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> f_; // Non-linear State transition function
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h_; // Non-linear measurement function (Jacobian)

    long x_dim_; // Dimension of state vector
    long H_dim_; // Dimenstion of measurement matrix

    // const float eps = 1e-4;
};


#endif  // EKF_HPP_