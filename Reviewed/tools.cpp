#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

#define EPS 0.001

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    VectorXd rmse(4);
	rmse << 0,0,0,0;

	if((estimations.size() != ground_truth.size()) || (estimations.size() == 0))
	{
		cout << "invalid estimation and ground_truth data " << endl;
		return rmse;
	}
	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    MatrixXd Hj(3,4);
	//recover state parameters
	double px = x_state(0);
	double py = x_state(1);
	double vx = x_state(2);
	double vy = x_state(3);

	//pre-compute a set of terms to avoid repeated calculation
	const double c1 = std::max(EPS,px*px+py*py);
	double c2 = sqrt(c1);
	double c3 = (c1*c2);

		
	//compute the Jacobian matrix
	Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

	return Hj;
}

float Tools::NormalizeAngle(float ang){
	while(ang > M_PI)
		ang = ang - 2 * M_PI;

	while(ang < - M_PI)
		ang = ang + 2 * M_PI;
	
	return ang;
}
