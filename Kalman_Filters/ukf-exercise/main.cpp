#include <iostream>
#include "Dense"
#include "ukf.h"

using Eigen::MatrixXd;

int main() {

  // Create a UKF instance
  UKF ukf;

  /**
   * Generate sigma points
   */
  MatrixXd Xsig1 = MatrixXd(5, 11);
  ukf.GenerateSigmaPoints(&Xsig1);
  // print result
  std::cout << "Xsig1 = " << std::endl << Xsig1 << std::endl;

  /**
   * Generate sigma points for augmented UKF
   */
  MatrixXd Xsig_aug = MatrixXd(7, 15);
  ukf.AugmentedSigmaPoints(&Xsig_aug);
  // print result
  std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;

  /**
   * Predict sigma points
   */
  MatrixXd Xsig_pred = MatrixXd(15, 5);
  ukf.SigmaPointPrediction(&Xsig_pred);
  // print result
  std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

  return 0;
}