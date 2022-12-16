/**
 * This module contains the class managing the ROS Logistics
 */

#ifndef ROS_LOGISTICS_HPP
#define ROS_LOGISTICS_HPP

#include "KalmanFilter.hpp"
#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/gazebo_client.hh>
#include <memory>
#include <vector>

namespace turtlebot3
{
	class ROS_Logistics
	{
	public:
		ROS_Logistics(std::shared_ptr<KalmanFilter>& turtlebot3_kf);

		~ROS_Logistics() = default;

		void callback_laser_scan(const sensor_msgs::LaserScan::ConstPtr& msg);

		void callback_odom(const nav_msgs::Odometry::ConstPtr& msg);

		void callback_sonar(const sensor_msgs::RangeConstPtr& msg);

		void callback_boxX_groundTruth(const nav_msgs::Odometry::ConstPtr& msg);

		void callback_sonar_gz(ConstSonarPtr& msg);

	private:
        std::shared_ptr<ros::NodeHandle> m_node;	// ptr cuz we shouldn't create it before calling ros::init
		gazebo::transport::NodePtr m_gazebo_node;

        std::shared_ptr<KalmanFilter> m_turtlebot3_kf;

		ros::Subscriber m_sub_laserScan;
		ros::Subscriber m_sub_odom;
		ros::Subscriber m_sub_sonar;

		gazebo::transport::SubscriberPtr m_sub_sonar_gz;

		ros::Publisher m_pub_boxX_groundTruth;
		ros::Publisher m_pub_laserx;
		ros::Publisher m_pub_laserx_kf;
		ros::Publisher m_pub_rmse;

		ros::Publisher m_pub_vel_boxX_groundTruth;
		ros::Publisher m_pub_vel_laserx_kf;
		ros::Publisher m_pub_vel_rmse;


		/* Establish necessary services, clients, publisher, listener communication */
		void setUp_ROS_Communication();
		void setUp_Gazebo_Communication();

		double odom_pos;
		double boxX_pos;
		double boxX_prevPos;	// for computing vel
		double laser_dist;
		std::vector<double> laser_dist_vec;

		int counter;
		std::vector<int> counter_vec;

		double box_prev_timestamp;

		std_msgs::Float64 boxX_gt;
		std_msgs::Float64 vel_boxX_gt;
		std_msgs::Float64 laser_dist_x;
		std_msgs::Float64 laser_dist_x_kf;
		std_msgs::Float64 laser_dist_vel_x_kf;
		std_msgs::Float64 m_rmse;
		std_msgs::Float64 m_vel_rmse;
		VectorXd x_out;


		Eigen::VectorXd x_;
	};


}    	// namespace turtlebot3

#endif	// #ifndef ROS_LOGISTICS_HPP
