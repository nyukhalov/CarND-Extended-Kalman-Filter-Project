#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;

  ekf_ = KalmanFilter();
  tools = Tools();
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  long long elapsed = measurement_pack.timestamp_ - previous_timestamp_;
  previous_timestamp_ = measurement_pack.timestamp_;

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    VectorXd init_state(4);
    MatrixXd init_P(4, 4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      cout << "Initializing state using radar measurement" << endl;
      // Convert radar from polar to cartesian coordinates and initialize state.
      float range = measurement_pack.raw_measurements_(0);
      float bearing = measurement_pack.raw_measurements_(1);

      float px = range * cos(bearing);
      float py = range * sin(bearing);

      init_state << px, py, 0, 0;
      init_P << 1, 0, 0,    0,
                0, 1, 0,    0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;
   }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      cout << "Initializing state using laser measurement" << endl;
      float px = measurement_pack.raw_measurements_(0);
      float py = measurement_pack.raw_measurements_(1);

      init_state << px, py, 0, 0;
      init_P << 1, 0, 0,    0,
                0, 1, 0,    0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;
    }

    ekf_.Init(init_state, init_P);

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  float dt = elapsed / 1000000.0;
  cout << "Elapsed sec: " << dt << endl;
  cout << "Predict state" << endl;
  ekf_.Predict(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    cout << "Update radar" << endl;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    cout << "Update laser" << endl;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
