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
    // Constructor && Destructor
    KF();
    ~KF();

    void GetMatrix(const Eigen::MatrixXd& matrix) {
      std::cout << "Matrix:\n" << matrix << std::endl;
    }
    void GetMatrices() {
      std::cout << "State Matrix:\n" << "Add matrix" << std::endl;
      std::cout << "State Matrixy:\n" << "Add matrix" << std::endl;
      std::cout << "State Matrix:\n" << "Add matrix" << std::endl;
      std::cout << "State Matrix:\n" << "Add matrix" << std::endl;
      std::cout << "State Matrix:\n" << "Add matrix" << std::endl;
      std::cout << "State Matrix:\n" << "Add matrix" << std::endl;
    }

    // Matrices
    // double dt;
    const Eigen::MatrixXd& a;

  private:
};


#endif  // KF_HPP_