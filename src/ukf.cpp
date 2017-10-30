#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

double NormalizeAngle(double angle_rad) {
  while (angle_rad > M_PI) angle_rad -= 2.0*M_PI;
  while (angle_rad < -M_PI) angle_rad += 2.0*M_PI;
  return angle_rad;
}

VectorXd StateDelta(const VectorXd& s1, const VectorXd& s2) {
  VectorXd delta = s1 - s2;
  delta[3] = NormalizeAngle(delta[3]);
  delta[4] = NormalizeAngle(delta[4]);
  return delta;
}

VectorXd RadarDelta(const VectorXd& s1, const VectorXd& s2) {
  VectorXd delta = s1 - s2;
  delta[1] = NormalizeAngle(delta[1]);
  return delta;
}

VectorXd LaserDelta(const VectorXd& s1, const VectorXd& s2) {
  return s1 - s2;
}

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

  P_ = 10 * MatrixXd::Identity(n_x_, n_x_);

  Xsig_pred_ = MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);
  weights_ = VectorXd::Zero(Xsig_pred_.cols());
  
  for(int i = 0; i < weights_.size(); i++) {
    if (i == 0) {
      weights_[i] = lambda_ / (lambda_ + n_aug_);
    } else {
      weights_[i] = 1 / (2 * (lambda_ + n_aug_));
    }
  }

  R_laser_ = MatrixXd(2, 2);
  R_laser_ <<
    std_laspx_*std_laspx_, 0,
    0, std_laspy_*std_laspy_;

  R_radar_ = MatrixXd(3, 3);
  R_radar_ <<
    std_radr_*std_radr_, 0, 0,
    0, std_radphi_*std_radphi_, 0,
    0, 0, std_radrd_*std_radrd_;

  std_a_ = 0.5;
  std_yawdd_ = M_PI / 6;
}

UKF::~UKF() {}

void UKF::Initialize(const MeasurementPackage& meas_package) {
  if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER) {
    x_[0] = meas_package.raw_measurements_[0];
    x_[1] = meas_package.raw_measurements_[1];
  } else {
    double rho = meas_package.raw_measurements_[0];
    double phi = meas_package.raw_measurements_[1];
    x_[0] = rho * cos(phi);
    x_[1] = rho * sin(phi);
  }
  x_[2] = 0;
  x_[3] = 0;
  x_[4] = 0;
}

MatrixXd UKF::GenerateSigmaPoints() {
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
  
  MatrixXd X_sigma = MatrixXd::Zero(n_aug_, 2*n_aug_ + 1);
  X_sigma.col(0) = x_aug;
  for(int i = 0; i < n_aug_; i++) {
    X_sigma.col(i+1) = x_aug + sigma_common.col(i);
    X_sigma.col(n_aug_+i+1) = x_aug - sigma_common.col(i);
  }

  return X_sigma;
}

void UKF::PredictSigmaPoints(const MatrixXd& X_sigma, double delta_t) {
  for(int i = 0; i < Xsig_pred_.cols(); i++) {
    const VectorXd& pt = X_sigma.col(i);
    double px = pt[0], py = pt[1], v = pt[2],
      yaw = pt[3], yaw_d = pt[4], v_acc = pt[5], yaw_acc = pt[6];
     
    VectorXd delta_x = VectorXd(5);
    VectorXd noise = VectorXd(5);
     
    if (fabs(yaw_d) > 1e-4) {
      delta_x(0) = v/yaw_d * (sin(yaw + yaw_d * delta_t) - sin(yaw));
      delta_x(1) = v/yaw_d * (-cos(yaw + yaw_d * delta_t) + cos(yaw));
    } else {
      delta_x(0) = v * cos(yaw) * delta_t;
      delta_x(1) = v * sin(yaw) * delta_t;
    }
     
    delta_x(2) = 0;
    delta_x(3) = yaw_d*delta_t;
    delta_x(4) = 0;
    
    noise(0) = (delta_t*delta_t)/2 * cos(yaw) * v_acc;
    noise(1) = (delta_t*delta_t)/2 * sin(yaw) * v_acc;
    noise(2) = delta_t * v_acc;
    noise(3) = yaw_acc*(delta_t*delta_t)/2;
    noise(4) = delta_t * yaw_acc;
     
    Xsig_pred_.col(i) = pt.head(5) + delta_x + noise;
  }
}

void UKF::PredictState() {
  VectorXd x = VectorXd::Zero(n_x_);
  MatrixXd P = MatrixXd::Zero(n_x_, n_x_);

  for(int i = 0; i < Xsig_pred_.cols(); i++) {
    x += weights_[i] * Xsig_pred_.col(i);
  }
  
  for(int i = 0; i < Xsig_pred_.cols(); i++) {
    VectorXd diff = StateDelta(Xsig_pred_.col(i), x);
    P += weights_[i] * diff * diff.transpose();
  }
  x_ = x;
  P_ = P;
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  double time_delta = (meas_package.timestamp_ - time_us_) / 1e6;
  time_us_ = meas_package.timestamp_;
  
  if (!is_initialized_) {
    Initialize(meas_package);
    is_initialized_ = true;
  } else {
    Prediction(time_delta);
    Update(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  MatrixXd X_sigma = GenerateSigmaPoints();
  PredictSigmaPoints(X_sigma, delta_t);
  PredictState();
}

void UKF::Update(const MeasurementPackage& meas_package) {
  MeasurementPackage::SensorType sensor = meas_package.sensor_type_;
  
  if (sensor == MeasurementPackage::LASER && use_laser_) {
    UpdateLidar(meas_package.raw_measurements_);
  } else if (sensor == MeasurementPackage::RADAR && use_radar_) {
    UpdateRadar(meas_package.raw_measurements_);
  }
}

void UKF::UpdateLidar(const VectorXd& z_meas) {
  MatrixXd Zsig = MatrixXd(2, Xsig_pred_.cols());
  VectorXd z_pred = VectorXd::Zero(2);
  MatrixXd S = MatrixXd::Zero(2, 2);
    
  for (int i = 0; i < Xsig_pred_.cols(); i++) {
    Zsig.col(i) = Xsig_pred_.col(i).head(2);
  }

  for (int i = 0; i < Zsig.cols(); i++) {
    z_pred += weights_[i] * Zsig.col(i);
  }

  for (int i = 0; i < Zsig.cols(); i++) {
    VectorXd diff = LaserDelta(Zsig.col(i), z_pred);
    S += weights_[i] * diff * diff.transpose();
  }

  S+= R_laser_;

  MatrixXd T = MatrixXd::Zero(n_x_, 2);
  for (int i = 0; i < Zsig.cols(); i++) {
    VectorXd x_diff = StateDelta(Xsig_pred_.col(i), x_);
    VectorXd z_diff = LaserDelta(Zsig.col(i), z_pred);
    T += weights_[i] * x_diff * z_diff.transpose();
  }

  VectorXd delta_z = LaserDelta(z_meas, z_pred);
  MatrixXd S_inv = S.inverse();
  
  MatrixXd K = T * S_inv;
  x_ = x_ + K * delta_z;
  P_ = P_ - K * S * K.transpose();

  VectorXd nis = delta_z.transpose() * S_inv * delta_z;
  cout << nis << endl;
  
}

void UKF::UpdateRadar(const VectorXd& z_meas) {
  MatrixXd Zsig = MatrixXd(3, Xsig_pred_.cols());
  VectorXd z_pred = VectorXd::Zero(3);
  MatrixXd S = MatrixXd::Zero(3, 3);
    
  for (int i = 0; i < Xsig_pred_.cols(); i++) {
    const VectorXd& pt = Xsig_pred_.col(i);
    double px = pt[0], py = pt[1], v = pt[2], yaw = pt[3];
    VectorXd z = VectorXd(3);

    z[0] = sqrt(px*px + py*py);;
    z[1] = atan2(py, px);
    z[2] = (px*cos(yaw)*v + py*sin(yaw)*v) / z[0];    
    Zsig.col(i) = z;
  }


  for (int i = 0; i < Zsig.cols(); i++) {
    z_pred += weights_[i] * Zsig.col(i);
  }

  for (int i = 0; i < Zsig.cols(); i++) {
    VectorXd diff = RadarDelta(Zsig.col(i), z_pred);
    S += weights_[i] * diff * diff.transpose();
  }

  S+= R_radar_;

  MatrixXd T = MatrixXd::Zero(n_x_, 3);
  for (int i = 0; i < Zsig.cols(); i++) {
    VectorXd x_diff = StateDelta(Xsig_pred_.col(i), x_);
    VectorXd z_diff = RadarDelta(Zsig.col(i), z_pred);
    T += weights_[i] * x_diff * z_diff.transpose();
  }

  VectorXd delta_z = RadarDelta(z_meas, z_pred);
  MatrixXd S_inv = S.inverse();

  MatrixXd K = T * S_inv;
  x_ = x_ + K * delta_z;
  P_ = P_ - K * S * K.transpose();

  VectorXd nis = delta_z.transpose() * S_inv * delta_z;
  cout << nis << endl;
}
