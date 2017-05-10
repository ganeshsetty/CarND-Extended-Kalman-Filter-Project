#include "kalman_filter.h"

#define pi 3.14159

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
  double rho,phi,rhodot;
	VectorXd h;
	rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
	phi = atan2(x_(1),x_(0));

	//normalize phi to be between -pi to pi
	while(phi > pi)
		phi = phi - 2 * pi;
	while(phi < - pi)
		phi = phi + 2 * pi;

	rhodot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
	h = VectorXd(3);
	h << rho,phi,rhodot;

	VectorXd y = z - h;
	//normalize phi to be between -pi to pi
	while(y(1) > pi)
		y(1) = y(1) - 2 * pi;

	while(y(1) < - pi)
		y(1) = y(1) + 2 * pi;

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
