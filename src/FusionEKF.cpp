#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

#define pi 3.14159

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

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ << 1,0,0,0,
		   0,1,0,0;


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
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
	float px,py,vx,vy;
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
	  float rho,phi,rhodot;
    	rho = measurement_pack.raw_measurements_(0); // range is distance to the object
    	phi = measurement_pack.raw_measurements_(1); // bearing
    	//check phi and normalize
    	while (phi > pi){
    		phi = phi - 2*pi;
    	}
    	while (phi < -pi){
    		phi = phi + 2*pi;
    	}

    	rhodot = measurement_pack.raw_measurements_(2); // range dot

    	px = rho * cos(phi);  // position x
    	py = rho * sin(phi);  // position y
    	vx = rhodot * cos(phi); // velocity x
    	vy = rhodot * sin(phi); // velocity y

    	ekf_.x_ << px,py,vx,vy;


    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
	  px = measurement_pack.raw_measurements_(0);
      py = measurement_pack.raw_measurements_(1);
	
	  //vx and vy from laser measurements are unknown(LIDAR don't provide velocity info)
      //and hence initialized with zeros
      ekf_.x_ << px,py,0,0;
    }
    // Initialize P the state covariance matrix
    ekf_.P_ = MatrixXd(4,4);

    ekf_.P_ << 1,0,0,0,
    	 0,1,0,0,
    	 0,0,1000,0,
    	 0,0,0,1000;

    previous_timestamp_ = measurement_pack.timestamp_;
	
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
   
  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
	//Elapsed time calculation
  double del_t = measurement_pack.timestamp_ - previous_timestamp_;
  del_t = del_t/1000000.0; // convert microsec to sec

  //Update the state transition matrix F
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1,0,del_t,0,
		  0,1,0,del_t,
		  0,0,1,0,
		  0,0,0,1;

  //Update the process noise covariance matrix.
  ekf_.Q_ = MatrixXd(4,4);
  float del_t2,del_t3,del_t4,del_t4_4,del_t3_2;
  float noise_ax = 9.0;
  float noise_ay = 9.0;

  del_t2 = del_t * del_t; // del_t^2
  del_t3 = del_t2 * del_t; // del_t^3
  del_t4 = del_t3 *del_t; //del_t^4
  del_t4_4 = del_t4/4;
  del_t3_2 = del_t3/2;

  ekf_.Q_ << del_t4_4 * noise_ax,0,del_t3_2 * noise_ax,0,
             0,del_t4_4 * noise_ay,0,del_t3_2 * noise_ay,
             del_t3_2 * noise_ax, 0,del_t2 * noise_ax, 0,
             0, del_t3_2 * noise_ay, 0, del_t2 * noise_ay;

  //Update the timestamp
  previous_timestamp_ = measurement_pack.timestamp_;

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
	  ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
	  ekf_.R_ = R_radar_;
	  ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
	  ekf_.H_ = H_laser_;
	  ekf_.R_ = R_laser_;
	  ekf_.Update(measurement_pack.raw_measurements_);
  }


  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
