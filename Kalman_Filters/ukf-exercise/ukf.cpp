#include "ukf.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

UKF::UKF() {
  Init();
}

UKF::~UKF() {

}

void UKF::Init() {

}

/**
 * UKF::GenerateSigmaPoints:
 */
void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out) {

  // set state dimension
  int n_x = 5;

  // define spreading parameter
  double lambda = 3 - n_x;

  // set example state
  VectorXd x = VectorXd(n_x);
  x <<   5.7441,
         1.3800,
         2.2049,
         0.5015,
         0.3528;

  // set example covariance matrix
  MatrixXd P = MatrixXd(n_x, n_x);
  P <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
          -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
           0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
          -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
          -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

  // create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x, 2 * n_x + 1);

  // calculate square root of P
  MatrixXd A = P.llt().matrixL();

  /**
   * Student part begin
   */

  // set the 1st column of sigma points
  Xsig.col(0) = x;

  // calculate the rest of sigma points
  for (int i = 0; i < n_x; ++i) {
    Xsig.col(i + 1) = x + sqrt(lambda + n_x) * A.col(i);
    Xsig.col(i + 1 + n_x) = x - sqrt(lambda + n_x) * A.col(i);
  }

  /**
   * Student part end
   */

  // print result
  // std::cout << "Xsig = " << std::endl << Xsig << std::endl;

  // write result
  *Xsig_out = Xsig;
}

/**
 * expected result:
 * Xsig =
 *  5.7441  5.85768   5.7441   5.7441   5.7441   5.7441  5.63052   5.7441   5.7441   5.7441   5.7441
 *    1.38  1.34566  1.52806     1.38     1.38     1.38  1.41434  1.23194     1.38     1.38     1.38
 *  2.2049  2.28414  2.24557  2.29582   2.2049   2.2049  2.12566  2.16423  2.11398   2.2049   2.2049
 *  0.5015  0.44339 0.631886 0.516923 0.595227   0.5015  0.55961 0.371114 0.486077 0.407773   0.5015
 *  0.3528 0.299973 0.462123 0.376339  0.48417 0.418721 0.405627 0.243477 0.329261  0.22143 0.286879
 */



/**
 * UKF::AugmentedSigmaPoints:
 */
void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {

  // set state dimension
  int n_x = 5;

  // set augmented dimension
  int n_aug = 7;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a = 0.2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd = 0.2;

  // define spreading parameter
  double lambda = 3 - n_aug;

  // set example state
  VectorXd x = VectorXd(n_x);
  x <<   5.7441,
         1.3800,
         2.2049,
         0.5015,
         0.3528;

  // create example covariance matrix
  MatrixXd P = MatrixXd(n_x, n_x);
  P <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
          -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
           0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
          -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
          -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

  // create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

  /**
   * Student part begin
   */

  // create augmented mean state
  x_aug.head(5) = x;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5, 5) = P;
  P_aug(5, 5) = std_a * std_a;
  P_aug(6, 6) = std_yawdd * std_yawdd;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug; ++i) {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda + n_aug) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug) = x_aug - sqrt(lambda + n_aug) * L.col(i);
  }

  /**
   * Student part end
   */

  // print result
  // std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;

  // write result
  *Xsig_out = Xsig_aug;
}

/**
 * expected result:
 *  Xsig_aug =
 * 5.7441  5.85768   5.7441   5.7441   5.7441   5.7441   5.7441   5.7441  5.63052   5.7441   5.7441   5.7441   5.7441   5.7441   5.7441
 *   1.38  1.34566  1.52806     1.38     1.38     1.38     1.38     1.38  1.41434  1.23194     1.38     1.38     1.38     1.38     1.38
 * 2.2049  2.28414  2.24557  2.29582   2.2049   2.2049   2.2049   2.2049  2.12566  2.16423  2.11398   2.2049   2.2049   2.2049   2.2049
 * 0.5015  0.44339 0.631886 0.516923 0.595227   0.5015   0.5015   0.5015  0.55961 0.371114 0.486077 0.407773   0.5015   0.5015   0.5015
 * 0.3528 0.299973 0.462123 0.376339  0.48417 0.418721   0.3528   0.3528 0.405627 0.243477 0.329261  0.22143 0.286879   0.3528   0.3528
 *      0        0        0        0        0        0  0.34641        0        0        0        0        0        0 -0.34641        0
 *      0        0        0        0        0        0        0  0.34641        0        0        0        0        0        0 -0.34641
 */



/**
 * UKF::SigmaPointPrediction:
 */
void UKF::SigmaPointPrediction(MatrixXd* Xsig_out) {

  // set state dimension
  int n_x = 5;

  // set augmented dimension
  int n_aug = 7;

  // create example sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
  Xsig_aug <<
    5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
      1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
    2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
    0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
    0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
         0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
         0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

  // create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

  double delta_t = 0.1; // time diff in sec

  /**
   * Student part begin
   */

  // predict sigma points
  for (int i = 0; i < 2 * n_aug + 1; ++i) {
    // extract state vector elements
    double px = Xsig_aug(0, i);
    double py = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    // predict the states
    double px_pred, py_pred;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_pred = px + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_pred = py + v / yawd * (-1 * cos(yaw + yawd * delta_t) + cos(yaw));
    }
    else {
      px_pred = px + v * cos(yaw) * delta_t;
      py_pred = py + v * sin(yaw) * delta_t;
    }

    double v_pred = v;
    double yaw_pred = yaw + yawd * delta_t;
    double yawd_pred = yawd;

    // add noise
    px_pred += 0.5 * delta_t * delta_t * cos(yaw) * nu_a;
    py_pred += 0.5 * delta_t * delta_t * sin(yaw) * nu_a;
    v_pred += delta_t * nu_a;
    yaw_pred += 0.5 * delta_t * delta_t * nu_yawdd;
    yawd_pred += delta_t * nu_yawdd;

  // write predicted sigma points into right column
    Xsig_pred(0, i) = px_pred;
    Xsig_pred(1, i) = py_pred;
    Xsig_pred(2, i) = v_pred;
    Xsig_pred(3, i) = yaw_pred;
    Xsig_pred(4, i) = yawd_pred;
  }

  /**
   * Student part end
   */

  // print result
  // std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

  // write result
  *Xsig_out = Xsig_pred;
}

/**
 * expected result:
 * Xsig_pred =
 * 5.93553 6.06251 5.92217 5.9415 5.92361 5.93516 5.93705 5.93553 5.80832 5.94481 5.92935 5.94553 5.93589 5.93401 5.93553
 * 1.48939 1.44673 1.66484 1.49719 1.508 1.49001 1.49022 1.48939 1.5308 1.31287 1.48182 1.46967 1.48876 1.48855 1.48939
 * 2.2049 2.28414 2.24557 2.29582 2.2049 2.2049 2.23954 2.2049 2.12566 2.16423 2.11398 2.2049 2.2049 2.17026 2.2049
 * 0.53678 0.473387 0.678098 0.554557 0.643644 0.543372 0.53678 0.538512 0.600173 0.395462 0.519003 0.429916 0.530188 0.53678 0.535048
 * 0.3528 0.299973 0.462123 0.376339 0.48417 0.418721 0.3528 0.387441 0.405627 0.243477 0.329261 0.22143 0.286879 0.3528 0.318159
 */



/**
 * UKF::PredictMeanAndCovariance:
 */
void UKF::PredictMeanAndCovariance(VectorXd* x_out, MatrixXd* P_out) {

  // set state dimension
  int n_x = 5;

  // set augmented dimension
  int n_aug = 7;

  // define spreading parameter
  double lambda = 3 - n_aug;

  // create example matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  Xsig_pred <<
         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  // create vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);

  // create vector for predicted state
  VectorXd x = VectorXd(n_x);

  // create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x, n_x);


  /**
   * Student part begin
   */

  // set weights
  weights(0) = lambda / (lambda + n_aug);
  for (int i = 1; i < 2 * n_aug + 1; ++i) {
    weights(i) = 1 / (2 * lambda + 2 * n_aug);
  }

  // predict state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; ++i) {
    x = x + weights(i) * Xsig_pred.col(i);
  }

  // predict state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; ++i) {
    // state difference 
    VectorXd x_diff = Xsig_pred.col(i) - x;
    // normalize the yaw angle within -pi to + pi
    while (x_diff(3) > M_PI) {
      x_diff(3) -= 2.0 * M_PI;
    }
    while (x_diff(3) < -M_PI) {
      x_diff(3) += 2.0 * M_PI;
    }

    P = P + weights(i) * x_diff * x_diff.transpose();
  }

  /**
   * Student part end
   */

  // print result
  // std::cout << "Predicted state" << std::endl;
  // std::cout << x << std::endl;
  // std::cout << "Predicted covariance matrix" << std::endl;
  // std::cout << P << std::endl;

  // write result
  *x_out = x;
  *P_out = P;
}

/**
 * expected result x:
 * x =
 * 5.93637
 * 1.49035
 * 2.20528
 * 0.536853
 * 0.353577
 * expected result p:
 * P =
 * 0.00543425 -0.0024053 0.00341576 -0.00348196 -0.00299378
 * -0.0024053 0.010845 0.0014923 0.00980182 0.00791091
 * 0.00341576 0.0014923 0.00580129 0.000778632 0.000792973
 * -0.00348196 0.00980182 0.000778632 0.0119238 0.0112491
 * -0.00299378 0.00791091 0.000792973 0.0112491 0.0126972
*/



/**
 * UKF::PredictRadarMeasurement:
 */
void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out) {

  // set state dimension
  int n_x = 5;

  // set augmented dimension
  int n_aug = 7;

  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  // define spreading parameter
  double lambda = 3 - n_aug;

  // set vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);
  double weight_0 = lambda/(lambda+n_aug);
  double weight = 0.5/(lambda+n_aug);
  weights(0) = weight_0;

  for (int i=1; i<2*n_aug+1; ++i) {
    weights(i) = weight;
  }

  // radar measurement noise standard deviation radius in m
  double std_radr = 0.3;

  // radar measurement noise standard deviation angle in rad
  double std_radphi = 0.0175;

  // radar measurement noise standard deviation radius change in m/s
  double std_radrd = 0.1;

  // create example matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  Xsig_pred <<
         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  /**
   * Student part begin
   */

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug + 1; ++i) {
    // extract state vector elements
    double px = Xsig_pred(0, i);
    double py = Xsig_pred(1, i);
    double v = Xsig_pred(2, i);
    double yaw = Xsig_pred(3, i);

    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    // measurement model
    Zsig(0, i) = sqrt(px * px + py * py);
    Zsig(1, i) = atan2(py, px);
    Zsig(2, i) = (px * v1 + py * v2) / sqrt(px * px + py*py);
  }

  // calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; ++i) {
    z_pred = z_pred + weights(i) * Zsig.col(i);
  }

  // calculate innovation covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; ++i) {
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    while (z_diff(1) > M_PI) {
      z_diff(1) -= 2.0 * M_PI;
    }
    while (z_diff(1) < -M_PI) {
      z_diff(1) += 2.0 * M_PI;
    }

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R <<  std_radr * std_radr, 0, 0,
        0, std_radphi * std_radphi, 0,
        0, 0,std_radrd * std_radrd;

  S = S + R;

  /**
   * Student part end
   */

  // print result
  // std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  // std::cout << "S: " << std::endl << S << std::endl;

  // write result
  *z_out = z_pred;
  *S_out = S;
}

/**
 * expected result z_out:
 * z_pred =
 * 6.12155
 * 0.245993
 * 2.10313
 * expected result s_out:
 * S =
 * 0.0946171 -0.000139448 0.00407016
 * -0.000139448 0.000617548 -0.000770652
 * 0.00407016 -0.000770652 0.0180917
 */



/**
 * UKF::UpdateState: 
 */
void UKF::UpdateState(VectorXd* x_out, MatrixXd* P_out) {

  // set state dimension
  int n_x = 5;

  // set augmented dimension
  int n_aug = 7;

  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  // define spreading parameter
  double lambda = 3 - n_aug;

  // set vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);
  double weight_0 = lambda/(lambda+n_aug);
  double weight = 0.5/(lambda+n_aug);
  weights(0) = weight_0;

  for (int i=1; i<2*n_aug+1; ++i) {  
    weights(i) = weight;
  }

  // create example matrix with predicted sigma points in state space
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  Xsig_pred <<
     5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
       1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
      2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
     0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
      0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  // create example vector for predicted state mean
  VectorXd x = VectorXd(n_x);
  x <<
     5.93637,
     1.49035,
     2.20528,
    0.536853,
    0.353577;

  // create example matrix for predicted state covariance
  MatrixXd P = MatrixXd(n_x,n_x);
  P <<
    0.0054342,  -0.002405,  0.0034157, -0.0034819, -0.00299378,
    -0.002405,    0.01084,   0.001492,  0.0098018,  0.00791091,
    0.0034157,   0.001492,  0.0058012, 0.00077863, 0.000792973,
   -0.0034819,  0.0098018, 0.00077863,   0.011923,   0.0112491,
   -0.0029937,  0.0079109, 0.00079297,   0.011249,   0.0126972;

  // create example matrix with sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);
  Zsig <<
    6.1190,  6.2334,  6.1531,  6.1283,  6.1143,  6.1190,  6.1221,  6.1190,  6.0079,  6.0883,  6.1125,  6.1248,  6.1190,  6.1188,  6.12057,
   0.24428,  0.2337, 0.27316, 0.24616, 0.24846, 0.24428, 0.24530, 0.24428, 0.25700, 0.21692, 0.24433, 0.24193, 0.24428, 0.24515, 0.245239,
    2.1104,  2.2188,  2.0639,   2.187,  2.0341,  2.1061,  2.1450,  2.1092,  2.0016,   2.129,  2.0346,  2.1651,  2.1145,  2.0786,  2.11295;

  // create example vector for mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred <<
      6.12155,
     0.245993,
      2.10313;

  // create example matrix for predicted measurement covariance
  MatrixXd S = MatrixXd(n_z,n_z);
  S <<
      0.0946171, -0.000139448,   0.00407016,
   -0.000139448,  0.000617548, -0.000770652,
     0.00407016, -0.000770652,    0.0180917;

  // create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
  z <<
     5.9214,   // rho in m
     0.2187,   // phi in rad
     2.0062;   // rho_dot in m/s

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x, n_z);

  /**
   * Student part begin
   */

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; ++i) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1) > M_PI) {
      z_diff(1) -= 2.0 * M_PI;
    }
    while (z_diff(1) < -M_PI) {
      z_diff(1) += 2.0 * M_PI;
    }

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x;
    // angle normalization
    while (x_diff(3) > M_PI) {
      x_diff(3) -= 2.0 * M_PI;
    }
    while (x_diff(3) < -M_PI) {
      x_diff(3) += 2.0 * M_PI;
    }

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  // calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = z - z_pred;
  // angle normalization
  while (z_diff(1) > M_PI) {
    z_diff(1) -= 2.0 * M_PI;
  }
  while (z_diff(1) < -M_PI) {
    z_diff(1) += 2.0 * M_PI;
  }

  // update state mean and covariance matrix
  x = x + K * z_diff;
  P = P - K * S * K.transpose();

  /**
   * Student part end
   */

  // print result
  // std::cout << "Updated state x: " << std::endl << x << std::endl;
  // std::cout << "Updated state covariance P: " << std::endl << P << std::endl;

  // write result
  *x_out = x;
  *P_out = P;
}

/**
 * expected result x:
 * x =
 * 5.92276
 * 1.41823
 * 2.15593
 * 0.489274
 * 0.321338
 * expected result P:
 * P =
 * 0.00361579 -0.000357881 0.00208316 -0.000937196 -0.00071727
 * -0.000357881 0.00539867 0.00156846 0.00455342 0.00358885
 * 0.00208316 0.00156846 0.00410651 0.00160333 0.00171811
 * -0.000937196 0.00455342 0.00160333 0.00652634 0.00669436
 *-0.00071719 0.00358884 0.00171811 0.00669426 0.00881797
 */
