#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // measurement matrix
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  /**
   * Finish initializing the FusionEKF.
   * Set the process and measurement noises
   */
  // according to Q and R
  // Q = E[v * v.transpose()]     // Expectation
//  Eigen::Vector4d v;                          // process noise
//  Eigen::Vector2d w_laser(0.15, 0.15);        // laser measurement noise
//  Eigen::Vector3d w_radar(0.3, 0.03, 0.3);    // Radar measurement noise

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * Initialize the state ekf_.x_ with the first measurement.     i.e. set up the prior
     * Create the state covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = Eigen::Vector4d(0, 0, 0, 0);

    ekf_.P_ = Eigen::Matrix4d();    // initial state covariance
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1000, 0,
               0, 0, 0, 1000;

    if (measurement_pack.sensor_type_ == SensorType::RADAR) {
      // Convert radar measurement from Polar to Cartesian coordinates and initialize state.
      auto rho = measurement_pack.raw_measurements_[0];
      auto phi = measurement_pack.raw_measurements_[1];     // bearing angle
      auto rho_dot = measurement_pack.raw_measurements_[2];
      ekf_.x_[0] = rho * cos(phi);
      ekf_.x_[1] = rho * sin(phi);
      ekf_.x_[2] = rho_dot * cos(phi);
      ekf_.x_[3] = rho_dot * sin(phi);
    }
    else if (measurement_pack.sensor_type_ == SensorType::LASER) {
      // Initialize state.
      ekf_.x_[0] = measurement_pack.raw_measurements_[0];
      ekf_.x_[1] = measurement_pack.raw_measurements_[1];
    }
    ekf_.x_[0] = (fabs(ekf_.x_[0]) > 0.00001) ? ekf_.x_[0] : (ekf_.x_[0] / fabs(ekf_.x_[0])) * 0.00001;
    ekf_.x_[1] = (fabs(ekf_.x_[1]) > 0.00001) ? ekf_.x_[1] : (ekf_.x_[1] / fabs(ekf_.x_[1])) * 0.00001;

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * Update the state transition matrix F according to the new elapsed time.
   * Update the process noise covariance matrix.(Q)
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;      // Time is measured in seconds.
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.F_ = Eigen::Matrix4d();
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;

  double noise_ax = 9;
  double noise_ay = 9;
  double dt2 = dt * dt;
  double dt3 = dt2 * dt;
  double dt4 = dt3 * dt;
  ekf_.Q_ = Eigen::Matrix4d();
  ekf_.Q_ << dt4 / 4 * noise_ax, 0, dt3 / 2 * noise_ax, 0,
             0, dt4 / 4 * noise_ay, 0, dt3 / 2 * noise_ay,
             dt3 / 2 * noise_ax, 0, dt2 * noise_ax, 0,
             0, dt3 / 2 * noise_ay, 0, dt2 * noise_ay;

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * - Use the sensor type to perform the update step.
   * - Update the state and state covariance matrices.
   */

  if (measurement_pack.sensor_type_ == SensorType::RADAR) {
    // Radar updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);     // H_j
    ekf_.R_ = R_radar_;

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;

    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
