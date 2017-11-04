#include "kalman_filter.h"
#include <iostream>

#define PI 3.14159265

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  I_ = Eigen::MatrixXd::Identity(4,4);
}

void KalmanFilter::Predict() {
  /** Last modified 20171103
  TODO:
    * predict the state
  */
  // x' = Fx+noise
  x_ = F_*x_;
  
  // P' = FPF_T+Q
  MatrixXd Ft = F_.transpose();
  P_ = F_*P_*Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /** Last Modified 20171103
  TODO:
    * update the state by using Kalman Filter equations
  */

  VectorXd y_ = z - H_ *x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S_ = H_*P_*Ht + R_;
  MatrixXd Si = S_.inverse();
  MatrixXd K_ = P_*Ht*Si;
  //  cout << "R_laser_: " << R_ << endl;
  // New state
  x_ = x_ + (K_*y_);
  P_ = (I_ - K_*H_ )*P_;
  
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /** Last Modified 20171103
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  // avoid divide by zero cases
  if(px == 0.0 && py == 0.0)
    return;

  float rho = sqrt(px*px + py*py);
  float theta = atan2(py,px);
  float rho_dot = (px*vx + py*vy) / rho;

  // h(x')
  VectorXd h(3);
  h << rho, theta, rho_dot;
  
  Hj_ = tools.CalculateJacobian(x_);

  //Update
  VectorXd y = z- h;
  // Doesn't work if I add or subtract 2PI
  while(y[1] > PI || y[1] < -PI) {
    if(y[1] > PI)
      y[1] -= PI;
    else
      y[1] += PI;
  }

  MatrixXd Hjt = Hj_.transpose();
  MatrixXd S = Hj_*P_*Hjt + R_; // SHould this be an update of the Radar R matrix?
  MatrixXd Si = S.inverse();
  MatrixXd K = P_*Hjt*Si;
  //cout << "R_radar_: " << R_ << endl;
  // New State
  x_ = x_ + (K*y);
  P_ = (I_ - K*Hj_) * P_;
}
