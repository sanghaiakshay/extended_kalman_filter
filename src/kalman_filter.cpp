#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
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
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft_ = F_.transpose();
  P_ = F_ * P_ * Ft_ + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd y_ = z - H_ * x_;
  MatrixXd Ht_ = H_.transpose();
  MatrixXd S_ = H_ * P_ * Ht_ + R_;
  MatrixXd Si_ = S_.inverse();
  MatrixXd K_ =  P_ * Ht_ * Si_;

  // new state
  x_ = x_ + (K_ * y_);
  int size = x_.size();
  MatrixXd I_ = MatrixXd::Identity(size, size);
  P_ = (I_ - K_ * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  // recover state parameters
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  float rho = sqrt(px*px + py*py);
  float phi = atan2(py,px);
  float rhodot;
  // check division by zero
  if (fabs(rho) < 0.0001) {
    rhodot = 0;
  }
  else{
    rhodot = (px*vx + py*vy)/rho;
  }
  
  VectorXd Hx_prime(3);
  Hx_prime << rho, phi, rhodot;
  
  VectorXd y_ = z - Hx_prime;
  while (y_(1)> M_PI) {
    y_(1) -= 2 * M_PI;
  }
  while (y_(1)< -M_PI) {
    y_(1) += 2 * M_PI;
  }
  MatrixXd Ht_ = H_.transpose();
  MatrixXd S_ = H_ * P_ * Ht_ + R_;
  MatrixXd Si_ = S_.inverse();
  MatrixXd K_ =  P_ * Ht_ * Si_;

  //new state
  x_ = x_ + (K_ * y_);
  int size = x_.size();
  MatrixXd I_ = MatrixXd::Identity(size, size);
  P_ = (I_ - K_ * H_) * P_;
}
