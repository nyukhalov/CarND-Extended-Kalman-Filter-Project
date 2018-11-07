#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
  tools = Tools();
}

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
  VectorXd h = h_radar();
  VectorXd y = z - h;

  double phi = y(1);
  double phi_norm = tools.NormAngleRad(phi);
  y(1) = phi_norm;

  MatrixXd Hj = tools.CalculateJacobian(x_);
  MatrixXd Hj_T = Hj.transpose();

  MatrixXd S = Hj * P_ * Hj_T + R_radar_;
  MatrixXd K = P_ * Hj_T * S.inverse();

  x_ = x_ + K * y;
  P_ = (I_ - K * Hj) * P_;
}

VectorXd KalmanFilter::h_radar() {
  VectorXd ret(3);

  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  if (abs(px * py) < 0.000001) {
    std::cout << "h_radar(): value of px/py is too low";
    throw "h_radar(): value of px/py is too low";
  }

  float h1 = sqrt(px*px + py*py);
  float h2 = atan2(py, px);
  float h3 = (px*vx + py*vy) / h1;

  ret << h1, h2, h3;

  return ret;
}
