#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in) {
  x_ = x_in;
  P_ = P_in;

  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  H_laser_T_ = H_laser_.transpose();

  //measurement covariance matrix - laser
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09,   0,    0,
              0, 0.0009,    0,
              0,      0, 0.09;

  Q_ = MatrixXd(4, 4);
  Q_ << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;

  I_ = MatrixXd::Identity(4, 4);
}

void KalmanFilter::Predict(float dt) {
  float noise_ax = 9;
  float noise_ay = 9;

  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt2 * dt2;

  // updating process noise matrix
  Q_(0, 0) = noise_ax * dt4 / 4.0;
  Q_(0, 1) = 0;
  Q_(0, 2) = noise_ax * dt3 / 2.0;
  Q_(0, 3) = 0;

  Q_(1, 0) = 0;
  Q_(1, 1) = noise_ay * dt4 / 4.0;
  Q_(1, 2) = 0;
  Q_(1, 3) = noise_ay * dt3 / 2.0;

  Q_(2, 0) = noise_ax * dt3 / 2.0;
  Q_(2, 1) = 0;
  Q_(2, 2) = noise_ax * dt2;
  Q_(2, 3) = 0;

  Q_(3, 0) = 0;
  Q_(3, 1) = noise_ay * dt3 / 2.0;
  Q_(3, 2) = 0;
  Q_(3, 3) = noise_ay * dt2;

  // updating state transition matrix
  F_(0, 2) = dt;
  F_(1, 3) = dt;

  // prediction
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_laser_ * x_;
  MatrixXd S = H_laser_ * P_ * H_laser_T_ + R_laser_;
  MatrixXd K = P_ * H_laser_T_ * S.inverse();

  x_ = x_ + K * y;
  P_ = (I_ - K * H_laser_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}
