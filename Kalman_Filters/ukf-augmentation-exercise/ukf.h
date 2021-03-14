#ifndef UKF_H
#define UKF_H

#include "Dense"

class UKF {
 public:
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * Init Initializes Unscented Kalman filter
   */
  void Init();

  /**
   * Student assignment functions
   */
  void GenerateSigmaPoints(Eigen::MatrixXd* Xsig_out);
  void AugmentedSigmaPoints(Eigen::MatrixXd* Xsig_out);
  void SigmaPointPrediction(Eigen::MatrixXd* Xsig_out);
  void PredictMeanAndCovariance(Eigen::VectorXd* x_pred, 
                                Eigen::MatrixXd* P_pred);
  void PredictRadarMeasurement(Eigen::VectorXd* z_out, 
                               Eigen::MatrixXd* S_out);
  void UpdateState(Eigen::VectorXd* x_out, 
                   Eigen::MatrixXd* P_out);
};

#endif  // UKF_H