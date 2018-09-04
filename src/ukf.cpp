
 /* ---- ADDED -----------*/

  
  /* ---- END ADDED -------*/
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

  // state vector dimension
  n_x_ = 5;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial state covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;// TODO: Change this value

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;// TODO: Change this value
  
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
  
  // augmented state dimension
  n_aug_ = n_x_ + 2;

  // sigma point spreading factor
  lambda_ = 3 - n_aug_;// ...this value is somewhat arbitrary; could try changing slightly

  // sigma points dimension
  n_sig_ = 2*n_aug_ + 1;

  // set weights
  weights_ = VectorXd(n_sig_);
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int i = 1; i < n_sig_; i++) { 
      double weight = 0.5 / (n_aug_ + lambda_);
      weights_(i) = weight;
  }

  // process noise covariance matrix
  Q = MatrixXd(2, 2);
  Q <<  std_a_ * std_a_, 0,
        0, std_yawdd_ * std_yawdd_;

  MatrixXd Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // measurement noise for laser
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << std_laspx_ * std_laspx_, 0,
              0, std_laspy_ * std_laspy_;

  // measurement noise for radar
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_ * std_radr_, 0, 0,
              0, std_radphi_ * std_radphi_, 0,
              0, 0, std_radrd_ * std_radrd_;

}

UKF::~UKF() {}

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
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // initialize time
    time_us_ = meas_package.timestamp_;

    // initialize process covariance matrix
    P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;

    //radar    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
      // get values
      double rho = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      double v = meas_package.raw_measurements_(2);
      double px = rho * cos(phi);
      double py = rho * sin(phi);
      double yawd = 0;

      // update state vector values
      x_ << px, py, v, phi, yawd;
    }

    // laser
    if (measurement_pack.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
      // get values
      double px = meas_package.raw_measurements_(0);
      double py = meas_package.raw_measurements_(1);

      // update the state vector
      x_ << px, py, 0, 0, 0;
    }

    // done initializing
    is_initialized_ = true;
    return;
  }

  // update the timestamp
  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;  //delta_t in seconds
  time_us_ = meas_package.timestamp_;

  // prediction
  Prediction(delta_t);

  // update
  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    UpdateLidar(meas_package);
  }

  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
  }
}













void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  


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

