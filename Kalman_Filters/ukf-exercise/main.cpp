#include <iostream>
#include "Dense"
#include "ukf.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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

  /**
   * Predict mean and covariance
   */
  VectorXd x_pred = VectorXd(5);
  MatrixXd P_pred = MatrixXd(5, 5);
  ukf.PredictMeanAndCovariance(&x_pred, &P_pred);
  // print result
  std::cout << "Predicted state" << std::endl;
  std::cout << x_pred << std::endl;
  std::cout << "Predicted covariance matrix" << std::endl;
  std::cout << P_pred << std::endl;

  /**
   * Predict Radar measurement
   */  
  VectorXd z_out = VectorXd(3);
  MatrixXd S_out = MatrixXd(3, 3);
  ukf.PredictRadarMeasurement(&z_out, &S_out);
  // print result
  std::cout << "z_pred: " << std::endl << z_out << std::endl;
  std::cout << "S: " << std::endl << S_out << std::endl;

  /**
   * Update state
   */
  VectorXd x_out = VectorXd(5);
  MatrixXd P_out = MatrixXd(5, 5);
  ukf.UpdateState(&x_out, &P_out);
  // print result
  std::cout << "Updated state x: " << std::endl << x_out << std::endl;
  std::cout << "Updated state covariance P: " << std::endl << P_out << std::endl;

  return 0;
}