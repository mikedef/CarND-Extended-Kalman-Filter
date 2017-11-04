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

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /** Last modified 20171102
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  noise_ax = 9.0;
  noise_ay = 9.0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /** Last Modified 20171103
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    VectorXd x(4);
    x << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      cout << "EKF Initialized with Radar measurement" << endl;
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      // range 
      float rho = measurement_pack.raw_measurements_[0];
      // bearing
      float phi = measurement_pack.raw_measurements_[1];
      // velocity of rho
      float rho_dot = measurement_pack.raw_measurements_[2];
      // convert coordinates from polar to cartesian
      // x
      float px = rho * cos(phi);
      if ( px<0.0001 ){
	px = 0.0001;
      }
      // y
      float py = rho * sin(phi);
      if ( py<0.0001 ){
	py = 0.0001;
      }
      // vx
      float vx = rho_dot * cos(phi);
      // vy
      float vy = rho_dot * sin(phi);
      x << px, py, 0.f, 0.f; 
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /** Last Modified 20171102
      Initialize state.
      */
      cout << "EKF Initialized with Laser measurement" << endl;
      x << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0.f, 0.f;
    }

    // Save the first timestamp 
    previous_timestamp_ = measurement_pack.timestamp_;

    // Initialize initial state covariance matrix
    /* covariance of 1 for the x and y position, and a high covariance for the x and y velocites since these are unknown. 
     */    
    MatrixXd P(4,4);
    P << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1000, 0,
      0, 0, 0, 1000;

    // Initialize measurement matrix for the laser
    H_laser_ << 1, 0 , 0, 0,
      0, 1, 0, 0;

    // Initialize initial transition matrix
    MatrixXd F(4,4);
    F << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;

    // Initialize the process covariance with an empty matrix
    MatrixXd Q(4,4);
    
    // Initialize ekf_ with all initial data
    ekf_.Init(x, P, F, H_laser_, R_laser_, Q);
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /** Last modified 20171103
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  // Update the time from the previous measurement in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
  // Update previous_timestamp
  previous_timestamp_ = measurement_pack.timestamp_;

  // Update transition matrix F with new elapsed time
  if(dt > 0.0){
    ekf_.F_ << 1, 0, dt, 0,
      0, 1, 0, dt,
      0, 0, 1, 0,
      0, 0, 0, 1;
  }

  // Update process covariance matrix Q
  float dt2 = dt*dt;
  float dt3 = dt2*dt;
  float dt4 = dt3*dt;
  float dt4_4 = dt4/4.0;
  float dt3_2 = dt3/2.0;

  ekf_.Q_ << dt4_4*noise_ax, 0, dt3_2*noise_ax, 0,
    0, dt4_4*noise_ay, 0, dt3_2*noise_ay,
    dt3_2*noise_ax, 0, dt2*noise_ax, 0,
    0, dt3_2*noise_ay, 0, dt2*noise_ay;

  
    ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.R_ = R_radar_;
    //cout << "R_radar" << endl;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    //cout << "R_laser" << endl;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
