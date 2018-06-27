#include <math.h>
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

const float PI2 = 2 * M_PI;

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
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
 
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  
  UpdateCommon(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  
  x_ = Px
       Py
       Vx
       Vy
  */
  float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  float phi = atan2(x_(1), x_(0));
  float rho_dot;

  /*
  Avoid Divide by Zero throughout the Implementation
    Before and while calculating the Jacobian matrix Hj, 
    make sure your code avoids dividing by zero. 
    For example, both the x and y values might be zero or px*px + py*py might be close to zero. 
    What should be done in those cases?
  */

  if(rho < 0.000001)
    rho = 0.000001;

  rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
  
  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;
  VectorXd y = z - z_pred;

  /*
  Normalizing Angles
    In C++, atan2() returns values between -pi and pi. 
    When calculating phi in y = z - h(x) for radar measurements, 
    the resulting angle phi in the y vector should be adjusted so that it is between -pi and pi. 
    The Kalman filter is expecting small angle values between the range -pi and pi. 
    HINT: when working in radians, you can add 2π or subtract 2π until the angle is within the desired range.
  */

  while(y(1) > M_PI){
    y(1) -= PI2;
  }

  while(y(1) < -M_PI){
    y(1) += PI2;
  }

  UpdateCommon(y);
}


void KalmanFilter::UpdateCommon(const VectorXd &y) {
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}