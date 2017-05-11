#include "kalman_filter.h"
#include <iostream>
#include "tools.h"
// Used M_PI
//#define pi 3.14159
using namespace std;
#define EPS 0.001

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  P_ = F_ * P_ * F_.transpose() + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    VectorXd y = z - H_ * x_;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K =  P_ * Ht * Si;

	//new state
	x_ = x_ + (K * y);
	size_t N = x_.size();
	MatrixXd I = MatrixXd::Identity(N, N);
	P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  double rho,phi;
  VectorXd h;
		
	rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
	//Note atan2(0.0,0.0) returns 0 so no need to ensure x_(1) and x_(0) to be non zeros
	phi = atan2(x_(1),x_(0)); //atan2 o/p is normalized  (-pi,pi)
	
	
	const double rhodot = (x_(0)*x_(2) + x_(1)*x_(3))/std::max(rho,EPS);
	h = VectorXd(3);
	h << rho,phi,rhodot;

	VectorXd y = z - h;
	
	//normalize y(1) phi to be between -pi to pi
	Tools tools;
	float NormAngle = tools.NormalizeAngle(y(1));
	y(1) = NormAngle;
	
		
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K =  P_ * Ht * Si;

	//new state
	x_ = x_ + (K * y);
	size_t N = x_.size();
	MatrixXd I = MatrixXd::Identity(N, N);
	P_ = (I - K * H_) * P_;

}
