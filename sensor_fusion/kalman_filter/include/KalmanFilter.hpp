/**
 * This module contains the class computing the KalmanFilter  of the Scara Robot
 */

#ifndef TURTLEBOT3_KALMAN_FILTER_HPP
#define TURTLEBOT3_KALMAN_FILTER_HPP

#include "Utils.hpp"
#include <vector>
#include "Eigen/Dense"
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace turtlebot3
{
	class KalmanFilter
	{
	public:
		KalmanFilter ();

		~KalmanFilter () = default;

		void ProcessMeasurement(const VectorXd& x, double timestamp, VectorXd& x_out);

		/**
		* Prediction Predicts the state and the state covariance
		* using the process model
		* @param delta_T Time between k and k+1 in s
		*/
		void Predict();

		/**
		* Updates the state by using standard Kalman Filter equations
		* @param z The measurement at k+1
		*/
		void Update(const Eigen::VectorXd &z);

		/**
		* Updates the state by using Extended Kalman Filter equations
		* @param z The measurement at k+1
		*/
		void UpdateEKF(const Eigen::VectorXd &z);

		// state vector
		Eigen::VectorXd x_;

		// state covariance matrix
		Eigen::MatrixXd P_;

		// state transition matrix
		Eigen::MatrixXd F_;

		// process covariance matrix
		Eigen::MatrixXd Q_;

		// measurement matrix
		Eigen::MatrixXd H_laser_ ;

		// measurement covariance matrix
		Eigen::MatrixXd R_laser_;

		// identity matrix
		Eigen::MatrixXd I_;

		bool is_initialized_;

		double previous_timestamp_;
	};

}    	// namespace turtlebot3

#endif	// #ifndef TURTLEBOT3_KALMAN_FILTER_HPP
