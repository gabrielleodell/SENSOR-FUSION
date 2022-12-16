/**
 * This module contains the class computing the KalmanFilter of the Scara Robot
 */

#include "KalmanFilter.hpp"
#include <ros/param.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <map>
#include <iostream>
using namespace std;

namespace turtlebot3
{

	KalmanFilter::KalmanFilter()
	{
		// Get all link lengths
//		std::map<std::string,double> link_lengths;
//		ros::param::get(param_scara_links_length, link_lengths);
//		m_L1 = link_lengths["L1"];
//		m_L2 = link_lengths["L2"];
//		m_L3 = link_lengths["L3"];
//		m_L4 = link_lengths["L4"];

		// Get link width and axel_offset
//		ros::param::get(param_scara_links_width, m_W);
//		m_axel_offset = m_W/2;

		is_initialized_ = false;

		previous_timestamp_ = 0;

		// initializing matrices
		x_ = VectorXd(4);
		x_ << 0, 0, 0, 0;

		P_ = MatrixXd(4, 4);
		P_ <<  1, 0, 0, 	0,
			   0, 1, 0, 	0,
			   0, 0, 1000, 	0,
			   0, 0, 0, 	1000;

		// Initialize transition matrix
		F_ = MatrixXd(4, 4);
		F_ <<  1, 0, 0, 0,
			   0, 1, 0, 0,
			   0, 0, 1, 0,
			   0, 0, 0, 1;

		H_laser_ = MatrixXd(2, 4);
		H_laser_ << 1, 0, 0, 0,
				  	0, 1, 0, 0;

		R_laser_ = MatrixXd(2, 2);
		R_laser_ << 0.0225, 0,
				  	0, 0.0225;

		Q_ = Eigen::MatrixXd::Zero(4,4);

		I_ = Eigen::MatrixXd::Identity(4,4);
	}

	void KalmanFilter::ProcessMeasurement(const VectorXd& z, double timestamp, VectorXd& x_out) {
		if (!is_initialized_) {
			x_ << z[0], z[1], 0.0, 0.0;
			previous_timestamp_ = timestamp;
			is_initialized_ = true;
			return;						// done initializing, no need to predict or update
		}

		double dt = (timestamp - previous_timestamp_)/1000000.0;
		previous_timestamp_ = timestamp;

		double diff = abs(x_[0] - z[0])/dt;

//		cout << "******Diff****** " << diff << endl;
		if (diff > 600)
		{
//			cout << "******Reinit****** " << diff << endl;
			P_ <<  2, 0, 0, 	0,
				   0, 2, 0, 	0,
				   0, 0, 500, 	0,
				   0, 0, 0, 	500;
		}

		if( dt > 0.0 )
		{
			// Update the motion model matrix for a timestep dt.
			// We use a motion model with a constant velocity.
			F_(0,2) = dt;
			F_(1,3) = dt;

			// Update the process noise covariance matrix for a timestep dt.
			// Our motion model uses Gaussian random accelerations in the x and y directions.
			float dt2 = dt*dt;
			float dt3 = dt2*dt;
			float dt4 = dt3*dt;
			float dt4over4 = dt4/4.;
			float dt3over2 = dt3/2.;
			float noise_ax = 9.0;
			float noise_ay = 9.0;
			if (diff > 700)
			{
				noise_ax = 100;
				noise_ay = 100;
			}
			Q_ <<  dt4over4*noise_ax,                 0, dt3over2*noise_ax,                0,
					0, dt4over4*noise_ay,                 0, dt3over2*noise_ay,
					dt3over2*noise_ax,                 0,      dt2*noise_ax,                 0,
					0, dt3over2*noise_ay,                 0,      dt2*noise_ay;
			Predict();
		}

		/**
		* Update
		*/
		Update(z);

		// print the output
		x_out = x_;
//		cout << "KF: \n" << x_ << "\n" << P_ << endl;
	}


	void KalmanFilter::Predict() {
	  /**
	   * TODO: predict the state
	   */
	  x_ = F_*x_;
	  MatrixXd Ft = F_.transpose();
	  P_ = F_*P_*Ft;
//	  cout << "Predict: \n" << x_ << "\n" << P_ << endl;
	}


	void KalmanFilter::Update(const VectorXd &z) {
	  /**
	   * TODO: update the state by using Kalman Filter equations
	   */
	  VectorXd y = z - H_laser_*x_;
	  MatrixXd Ht = H_laser_.transpose();
	  MatrixXd S = H_laser_*P_*Ht + R_laser_;
	  MatrixXd Si = S.inverse();
	  MatrixXd K =  P_*Ht*Si;

	  // New state
	  x_ = x_ + ( K*y );
	  P_ = ( I_ - K*H_laser_ )*P_;
//	  cout << "Update: \n" << x_ << "\n" << P_ << endl;
	}


	void KalmanFilter::UpdateEKF(const VectorXd &z) {
	  /**
	   * TODO: update the state by using Extended Kalman Filter equations
	   */
	}

}    	// namespace turtlebot3
