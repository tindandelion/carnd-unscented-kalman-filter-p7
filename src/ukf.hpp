#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Helper class to calculate Kalman gain and update the state mean and covariance
 */
class MeasurementUpdate {
  VectorXd dz_;
  MatrixXd S_;
  MatrixXd T_;
  MatrixXd S_inv_;

public:
  MeasurementUpdate(const VectorXd& dz, const MatrixXd& S, const MatrixXd& T):
    dz_(dz), S_(S), T_(T), S_inv_(S.inverse()) { }
  
  void UpdateState(VectorXd& x, MatrixXd& P) const {
    MatrixXd K = T_ * S_inv_;
    x = x + K * dz_;
    P = P - K * S_ * K.transpose();
  }

  double Nis() const {
    VectorXd nis = dz_.transpose() * S_inv_ * dz_;
    return nis[0];
  }
};


/*
 * Main class to perform UKF predict/update cycle with laser or radar measurements
 */
class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;


  const double std_a_ = 0.5;  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  const double std_yawdd_ = M_PI / 6;   ///* Process noise standard deviation yaw acceleration in rad/s^2


  const double std_laspx_ = 0.15;   ///* Laser measurement noise standard deviation position1 in m
  const double std_laspy_ = 0.15;   ///* Laser measurement noise standard deviation position2 in m

  const double std_radr_ = 0.3;   ///* Radar measurement noise standard deviation radius in m
  const double std_radphi_ = 0.03;   ///* Radar measurement noise standard deviation angle in rad
  const double std_radrd_ = 0.3;   ///* Radar measurement noise standard deviation radius change in m/s

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* Noise matrices
  MatrixXd Q_;
  MatrixXd R_laser_;
  MatrixXd R_radar_;

  UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);
  
private:
  void Initialize(const MeasurementPackage& meas_package);
  void Prediction(double delta_t);
  MatrixXd GenerateSigmaPoints();
  void PredictSigmaPoints(const MatrixXd& X_sigma, double delta_t);
  void PredictState();
  
  void Update(const MeasurementPackage& meas_package);
  MeasurementUpdate CalcLidarUpdate(const VectorXd& z_meas) const;
  MeasurementUpdate CalcRadarUpdate(const VectorXd& z_meas) const;
  void UpdateState(const MeasurementUpdate& update);
};

#endif /* UKF_H */
