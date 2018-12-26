#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Please note that the Eigen library does not gauantee to initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 * It may be initialized with zeros but !!may not!!, which will cause huge error!
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * predict the state
   */
   x_ = F_ * x_;                          // simplified: ignore input(control) vector u
   P_ = F_ * P_ * F_.transpose() + Q_;    // acceleration is considered into Q_
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * update the state by using Kalman Filter equations
   */
  Eigen::Vector2d y = z - H_ * x_;

  UpdateWithY(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * update the state by using Extended Kalman Filter equations
   */
  // Mapping function from Cartesian coordinates to Polar coordinates
  Eigen::Vector3d h_x;
  h_x[0] = sqrt(x_[0] * x_[0] + x_[1] * x_[1]);
  h_x[1] = atan2(x_[1], x_[0]);         // [-pi, pi]
  h_x[2] = (x_[0] * x_[2] + x_[1] * x_[3]) / h_x[0];

  Eigen::Vector3d y = z - h_x;
  // wrap up phi to [-pi, pi]
  y[1] = fmod(y[1] - M_PI, 2 * M_PI) + M_PI;
  UpdateWithY(y);
}

void KalmanFilter::UpdateWithY(const Eigen::VectorXd &y) {
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());

  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;
}
