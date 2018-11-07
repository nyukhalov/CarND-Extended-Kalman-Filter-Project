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

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
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
    // first measurement
    cout << "EKF: " << endl;
    VectorXd init_state(4);
    MatrixXd init_P(4, 4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates and initialize state.
      init_state << 1, 1, 1, 1;
      init_P << 1, 0, 0,    0,
                0, 1, 0,    0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;
   }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
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
  ekf_.Predict(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
