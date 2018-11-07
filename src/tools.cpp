#include <iostream>
#include "tools.h"
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  if (estimations.size() == 0) throw "estimations must not be empty";
  if (estimations.size() != ground_truth.size()) throw "estimations and ground_truth sizes must be equal";
  if (estimations[0].size() != ground_truth[0].size()) throw "estimations and ground_truth dimentions must match";

  // vector dimention
  int dim = estimations[0].size();

  // accumulator initialized by zeros
  VectorXd acc(dim);
  for (int i=0; i<dim; i++) acc(i) = 0;

  // accumulating error
  for (int i=0; i<estimations.size(); i++) {
    VectorXd diff = estimations[i] - ground_truth[i];
    VectorXd diff2 = diff.array() * diff.array();
    acc = acc + diff2;
  }

  // computing RMSE
  VectorXd rmse = (acc / estimations.size()).array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  if (x_state.size() != 4) throw "Invalid state vector";

  float px = x_state[0];
  float py = x_state[1];
  float vx = x_state[2];
  float vy = x_state[3];

  float px2 = px * px;
  float py2 = py * py;
  float pxy2 = px2 + py2;
  float sqrt_pxy2 = sqrt(pxy2);

  if (pxy2 < 0.000001) {
    std::cout << "Low x or y value" << std::endl;
    throw "Low x or y value";
  }

  float e11 = px / sqrt_pxy2;
  float e12 = py / sqrt_pxy2;

  float e21 = -py / pxy2;
  float e22 = px / pxy2;

  float e31 = py * (vx*py - vy*px) / pow(pxy2, 3.0/2.0);
  float e32 = px * (vy*px - vx*py) / pow(pxy2, 3.0/2.0);
  float e33 = px / sqrt_pxy2;
  float e34 = py / sqrt_pxy2;

  MatrixXd Hj(3, 4);
  Hj << e11, e12, 0, 0,
        e21, e22, 0, 0,
        e31, e32, e33, e34;

  return Hj;
}

double Tools::NormAngleRad(double angle) {
  if (angle < 0) {
    while(angle < -M_PI) angle += 2 * M_PI;
  } else {
    while(angle > M_PI) angle -= 2 * M_PI;
  }
  return angle;
}
