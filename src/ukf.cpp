#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  
  n_x_ = 5;
  n_aug_ = n_x_ + 2;
  lambda_ = 3 - n_aug_;
  
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  is_initialized_ = false;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
}

UKF::~UKF() {}

void UKF::Initialize(const MeasurementPackage& meas_package) {
  if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER) {
    x_[0] = meas_package.raw_measurements_[0];
    x_[1] = meas_package.raw_measurements_[1];
    x_[2] = 0;
    x_[3] = 0;
    x_[4] = 0;
  } else {
    double rho = meas_package.raw_measurements_[0];
    double phi = meas_package.raw_measurements_[1];
    x_[0] = rho * cos(phi);
    x_[1] = rho * sin(phi);
    x_[2] = 0;
    x_[3] = 0;
    x_[4] = 0;
  }
}

void UKF::GenerateSigmaPoints(MatrixXd& X_sigma) {
  VectorXd x_aug = VectorXd::Zero(n_aug_);
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  MatrixXd Q = MatrixXd(2, 2);
  Q <<
    std_a_*std_a_, 0,
    0, std_yawdd_*std_yawdd_;
  
  x_aug.head(n_x_) = x_;
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug.bottomRightCorner(2, 2) = Q;

  MatrixXd P_sqrt = P_aug.llt().matrixL();
  MatrixXd sigma_common = sqrt(lambda_ + n_aug_) * P_sqrt;
  
  X_sigma = MatrixXd::Zero(n_aug_, 2*n_aug_ + 1);
  X_sigma.col(0) = x_aug;
  for(int i = 0; i < n_aug_; i++) {
    X_sigma.col(i+1) = x_aug + sigma_common.col(i);
    X_sigma.col(n_aug_+i+1) = x_aug - sigma_common.col(i);
  }
  
}

void UKF::PredictSigmaPoints(const MatrixXd& X_sigma, MatrixXd& X_sigma_pred, double delta_t) {
  X_sigma_pred = X_sigma;
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  double time_delta = (meas_package.timestamp_ - time_us_) / 1e6;
  time_us_ = meas_package.timestamp_;
  
  if (!is_initialized_) {
    Initialize(meas_package);
    is_initialized_ = true;
  } else {
    Prediction(time_delta);
  }

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  MatrixXd X_sigma, X_sigma_pred;
  GenerateSigmaPoints(X_sigma);
  PredictSigmaPoints(X_sigma, X_sigma_pred, delta_t);
  cout << "X_sigma_pred = " << X_sigma_pred << endl;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
