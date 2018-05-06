#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // constant for state dimensions
  n_x_ = 5;

  // initialize state vector
  x_ = VectorXd(n_x_);

  // initialize covariance matrix P
  P_ = MatrixXd(n_x_, n_x_);     // change the below matrix to initialize with 0.5
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // std_a_ = 30;
  std_a_ = 0.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  // std_yawdd_ = 30;
  std_yawdd_ = 0.5;

//  std_a_    std_yawdd__     x       y       vx      vy
//  1.5       0.5             0.0693  0.0835  0.3336  0.2380
//  30        30              0.0976  0.1209  0.8697  0.9845
//  15        15              0.0896  0.1100  0.6228  0.6670
//  5         5               0.0787  0.0945  0.4187  0.3792
//  2         2               0.0702  0.0858  0.3521  0.2693
//  1         1               0.0650  0.0840  0.3309  0.2342
//  0.5       0.5             0.0615  0.0862  0.3266  0.2283


  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  lambda_ = 3 - n_x_;       // design parameter, per class notes
  n_aug_ = n_x_ + 2;        // includes noise vector
  n_sig_ = 2 * n_aug_ + 1;  // number of sigma points, per class notes

  cout << "total number of sigma points: " << n_sig_ << endl;
  cout << "augmented vector dimensions:  "<< n_aug_ << endl;

  // Initialize weights for prediction measurements
  weights_ = VectorXd(n_sig_);
  weights_.fill(0.5 / (n_aug_ + lambda_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);

  // Initialize measurement noise covariance matrix
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_*std_radr_,  0,                        0,
              0,                    std_radphi_*std_radphi_,  0,
              0,                    0,                        std_radrd_*std_radrd_;

  R_lidar_ = MatrixXd(2, 2);
  R_lidar_ << std_laspx_*std_laspx_,  0,
              0,                      std_laspy_*std_laspy_;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_package) {

  if ( !is_initialized_) {

    time_us_ = measurement_package.timestamp_;

    if (measurement_package.sensor_type_ == MeasurementPackage::RADAR) {

      // read initial measurements
      double rho = measurement_package.raw_measurements_[0];
      double phi = measurement_package.raw_measurements_[1];
      double rho_dot = measurement_package.raw_measurements_[2];

      double x = rho * cos(phi);
      double y = rho * sin(phi);

      double vx = rho_dot * cos(phi);
  	  double vy = rho_dot * sin(phi);
      double v = sqrt(vx * vx + vy * vy);

      x_ << x, y, v, 0, 0;

    } else {

      x_ << measurement_package.raw_measurements_[0], measurement_package.raw_measurements_[1], 0, 0, 0;
    }

    // initialization complete
    is_initialized_ = true;
    return;
  }

  // time delta
  double dt = (measurement_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = measurement_package.timestamp_;

  // Prediction step
  Prediction(dt);

  if (use_radar_ && measurement_package.sensor_type_ == MeasurementPackage::RADAR) {
    // update measurements from RADAR
    UpdateRadar(measurement_package);
  } else if (use_laser_ && measurement_package.sensor_type_ == MeasurementPackage::LASER) {
    // update measurements from LIDAR
    UpdateLidar(measurement_package);
  } else {
    // not very likely
    return;
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

  // initialize augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);

  // initialize augmented stated with P & Q
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(5,5) = pow(std_a_, 2);
  P_aug(6,6) = pow(std_yawdd_, 2);

  Xsig_pred_ = ApplyMeasurementModel(x_aug, P_aug, delta_t);

  x_ = Xsig_pred_ * weights_;
  P_.fill(0.0);     // predicted state covariance matrix
  for (int i = 0; i < n_sig_; i++) {  //iterate over sigma points

    // state difference
    VectorXd xDiff = Xsig_pred_.col(i) - x_;
    // angle normalization
    NormalizeAngle(xDiff, 3);

    P_ = P_ + weights_(i) * xDiff * xDiff.transpose() ;
  }

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage measurement_package) {

  // predit measurement
  int n_z = 2;
  MatrixXd Zsig = Xsig_pred_.block(0, 0, n_z, n_sig_);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < n_sig_; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  S = S + R_lidar_;

  // Incoming radar measurement
  VectorXd z = measurement_package.raw_measurements_;

  //create matrix for cross correlation
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  Tc.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  //NIS Lidar Update
  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
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
  // Radar measurement dimension
  int n_z = 3;

  // 1. Predict measurement
  MatrixXd Zsig = MatrixXd(n_z, n_sig_);
  //transform sigma points into measurement space
  for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //rho
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < n_sig_; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    NormalizeAngle(z_diff, 1);

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  S = S + R_radar_;

  // 2. Update state
  // Incoming radar measurement
  VectorXd z = meas_package.raw_measurements_;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  Tc.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    NormalizeAngle(z_diff, 1);

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    NormalizeAngle(x_diff, 3);

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  NormalizeAngle(z_diff, 1);

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  //NIS Update
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
}

/**
 *  Normalize the vector component to be with in the limits of [-M_PI, M_PI]
 */
void UKF::NormalizeAngle(VectorXd vector, int index) {
  while (vector(index) > M_PI) {
    vector(index) -= 2.*M_PI;
  }

  while (vector(index) < -M_PI) {
    vector(index) += 2.*M_PI;
  }
}


/**
 * Apply measurement model and return prediction for every instant in time
 * @param x
 * @param P
 * @param delta_t
 * @return
 */

MatrixXd UKF::ApplyMeasurementModel(VectorXd x, MatrixXd P, double delta_t) {
  int n = x.size();

  MatrixXd Xsig = MatrixXd(n, n_sig_);            // sigma point matrix
  MatrixXd Xsig_pred = MatrixXd(n_x_, n_sig_);    // sigma prediction matrix
  MatrixXd A = P.llt().matrixL();                 // simplified form for square root of P

  Xsig.col(0) = x;

  double lmdXSqrt = sqrt(lambda_ + n);
  for (int i = 0; i < n; i++){
    Xsig.col( i + 1 ) = x + lmdXSqrt * A.col(i);
    Xsig.col( i + 1 + n ) = x - lmdXSqrt * A.col(i);
  }

  //prediction sigma points
  for (int i = 0; i< n_sig_; i++) {
    double px = Xsig(0,i);
    double py = Xsig(1,i);
    double v  = Xsig(2,i);
    double yaw  = Xsig(3,i);
    double yawd = Xsig(4,i);

    double noise_acc = Xsig(5,i);       //  longitudinal acceleration (m/s^2)
    double noise_yaw_acc = Xsig(6,i);   //  yaw acceleration (rad/s^2)

    //predicted state values
    double pxP, pyP;

    // handle divide by zero issues.
    if (fabs(yawd) > 0.001) {
      pxP = px + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
      pyP = py + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
      pxP = px + v*delta_t*cos(yaw);
      pyP = py + v*delta_t*sin(yaw);
    }

    double vP = v;
    double yawP = yaw + yawd*delta_t;
    double yawdP = yawd;

    // adjustments for noise function
    pxP = pxP + 0.5*noise_acc*delta_t*delta_t * cos(yaw);
    pyP = pyP + 0.5*noise_acc*delta_t*delta_t * sin(yaw);
    vP = vP + noise_acc*delta_t;

    yawP = yawP + 0.5*noise_yaw_acc*delta_t*delta_t;
    yawdP = yawdP + noise_yaw_acc*delta_t;

    // predicted sigma point
    Xsig_pred(0,i) = pxP;
    Xsig_pred(1,i) = pyP;
    Xsig_pred(2,i) = vP;
    Xsig_pred(3,i) = yawP;
    Xsig_pred(4,i) = yawdP;
  }

  return Xsig_pred;
}