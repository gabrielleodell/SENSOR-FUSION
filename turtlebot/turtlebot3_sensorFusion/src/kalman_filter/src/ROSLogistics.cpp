/**
 * This module contains the class managing the ROS Logistics
 */

#include "ROSLogistics.hpp"
#include "Utils.hpp"
#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <chrono>
#include <cmath>
#include "Eigen/Dense"
#include "matplotlibcpp.h"
using namespace std::chrono;
namespace plt = matplotlibcpp;

#include <iostream>
using namespace std;

namespace turtlebot3
{

	ROS_Logistics::ROS_Logistics(std::shared_ptr<KalmanFilter>& turtlebot3_kf)	:	m_node(nullptr),
																					m_turtlebot3_kf(nullptr),
																					counter(0),
																					box_prev_timestamp(0.0)
	{
		// Register the node with ROS Master
		m_node = std::make_shared<ros::NodeHandle>();

		// Establish all necessary connections
		setUp_ROS_Communication();
//		setUp_Gazebo_Communication();	// not needed because sonar was figured out
		ROS_INFO("turtlebot3_kf all services and publishers up and running . . .");


		// Create ScaraKinematics object after brining up ros parameter server
		m_turtlebot3_kf = turtlebot3_kf;

		x_ = VectorXd(2);
		x_ << 0.0, 0.0;

		x_out = VectorXd(4);
		x_out << 0, 0, 0, 0;

		plt::ion();
		plt::show();

		// Keep looking into service, subscriber queues once in 100ms
		int spin_rate = 100; // 10 Hz = 1/10 sec = 10ms
		ros::Rate rate(spin_rate);
		while (m_node->ok())
		{
			gazebo::common::Time::MSleep(10);
			ros::spinOnce();
			rate.sleep();
		}
	}


	void ROS_Logistics::callback_laser_scan(const sensor_msgs::LaserScan::ConstPtr& msg)
	{
//		counter_vec.clear();
//		counter_vec.push_back(counter);
//		laser_dist_vec.clear();
//		laser_dist = msg->ranges.at(0);
//		cout << "Laser: " << laser_dist << endl;
//		plt::subplot(2, 1, 1);
//		plt::title("Laser_x");
//		laser_dist_vec.push_back(laser_dist);
//		plt::plot(counter_vec, laser_dist_vec, "seagreen");
//        plt::axis("equal");
//		plt::draw();
//		plt::pause(0.000000001);

		laser_dist = msg->ranges.at(0);
		if (isnan(laser_dist) || isinf(laser_dist))
		{
			laser_dist = msg->range_max;
		}
//		cout << "Laser: " << laser_dist << "\n" << endl;

		double laser_dist_y = msg->ranges.at(90);
		if (isnan(laser_dist_y) || isinf(laser_dist_y))
		{
			laser_dist_y = msg->range_max;
		}

		laser_dist_x.data = laser_dist;
		m_pub_laserx.publish(laser_dist_x);

		x_ << laser_dist, laser_dist_y;


		milliseconds ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
//		cout << "Time Inc: " <<  ms.count() << endl;

		m_turtlebot3_kf->ProcessMeasurement(x_, ms.count(), x_out);
		laser_dist_x.data = x_out[0];
		m_pub_laserx_kf.publish(laser_dist_x);


		double rmse = 0.0;
		double vel_rmse = 0.0;
		if (laser_dist >= msg->range_max)
		{
			laser_dist_vel_x_kf.data = 0.0;
			rmse = 0.0;
			vel_rmse = 0.0;
		}
		else
		{
			laser_dist_vel_x_kf.data = x_out[2];

			double err = (boxX_gt.data - x_out[0]);	// GroundTruth - KF_Estimate
			rmse = sqrt(pow(err, 2));

			double vel_err = (vel_boxX_gt.data - x_out[2]);	// GroundTruth - KF_Estimate
			vel_rmse = sqrt(pow(err, 2));
		}
		m_pub_vel_laserx_kf.publish(laser_dist_vel_x_kf);
		m_rmse.data = rmse;
		m_pub_rmse.publish(m_rmse);
		m_vel_rmse.data = vel_rmse;
		m_pub_vel_rmse.publish(m_vel_rmse);
	}


	void ROS_Logistics::callback_boxX_groundTruth(const nav_msgs::Odometry::ConstPtr& msg)
	{
		boxX_pos = msg->pose.pose.position.x;
		boxX_gt.data = boxX_pos - odom_pos;
		m_pub_boxX_groundTruth.publish(boxX_gt);

		milliseconds ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
		double timestamp = ms.count();
		double dt = (timestamp - box_prev_timestamp)/1000.0;
		double vel = abs(boxX_pos - boxX_prevPos)/dt;
//		cout << "vel>>>>>>> " << abs(boxX_pos - boxX_prevPos) << " -- " << dt << " = " << vel << endl;
		vel_boxX_gt.data = vel;
		m_pub_vel_boxX_groundTruth.publish(vel_boxX_gt);
		boxX_prevPos = boxX_pos;
		box_prev_timestamp = timestamp;
	}

	void ROS_Logistics::callback_sonar(const sensor_msgs::RangeConstPtr& msg)
	{
//		cout << "Sonar: " << msg->range << endl;

		x_ << msg->range, 0;
		milliseconds ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
//		cout << "Time Inc: " <<  ms.count() << endl;

		m_turtlebot3_kf->ProcessMeasurement(x_, ms.count(), x_out);
		laser_dist_x.data = x_out[0];
		m_pub_laserx_kf.publish(laser_dist_x);

		double rmse = 0.0;
		double vel_rmse = 0.0;
		if (msg->range >= msg->max_range)
		{
			rmse = 0.0;
			laser_dist_vel_x_kf.data = 0.0;
			vel_rmse = 0.0;
		}
		else
		{
			laser_dist_vel_x_kf.data = x_out[2];

			double err = (boxX_gt.data - x_out[0]);	// GroundTruth - KF_Estimate
			rmse = sqrt(pow(err, 2));

			double vel_err = (vel_boxX_gt.data - x_out[2]);	// GroundTruth - KF_Estimate
			vel_rmse = sqrt(pow(err, 2));
		}
		m_pub_vel_laserx_kf.publish(laser_dist_vel_x_kf);

		m_rmse.data = rmse;
		m_pub_rmse.publish(m_rmse);

		m_vel_rmse.data = vel_rmse;
		m_pub_vel_rmse.publish(m_vel_rmse);
	}

	void ROS_Logistics::callback_odom(const nav_msgs::Odometry::ConstPtr& msg)
	{
		odom_pos = msg->pose.pose.position.x;
//		cout << "Odom: " << odom_pos << "\n" << endl;
		boxX_pos = 4.8;
		boxX_gt.data = boxX_pos - odom_pos;
		m_pub_boxX_groundTruth.publish(boxX_gt);
	}


	/* Establish necessary services, clients, publisher, listener communication */
	void ROS_Logistics::setUp_ROS_Communication()
	{
		int queue_size = 1;
		string topic_laser_scan = "/scan";
		m_sub_laserScan = m_node->subscribe(topic_laser_scan, queue_size, &ROS_Logistics::callback_laser_scan, this);

		string topic_odom = "/odom";
		m_sub_odom = m_node->subscribe(topic_odom, queue_size, &ROS_Logistics::callback_odom, this);

		string topic_boxX = "/box_x";
//		m_sub_odom = m_node->subscribe(topic_boxX, queue_size, &ROS_Logistics::callback_boxX_groundTruth, this);

		string topic_sonar = "/sonar";
		m_sub_sonar = m_node->subscribe(topic_sonar, queue_size, &ROS_Logistics::callback_sonar, this);

		m_pub_boxX_groundTruth = m_node->advertise<std_msgs::Float64>("/boxX_groundTruth", queue_size);
		m_pub_laserx = m_node->advertise<std_msgs::Float64>("/laser_x", queue_size);
		m_pub_laserx_kf = m_node->advertise<std_msgs::Float64>("/laser_x_kf", queue_size);
		m_pub_rmse = m_node->advertise<std_msgs::Float64>("/kf_rmse", queue_size);

		m_pub_vel_boxX_groundTruth = m_node->advertise<std_msgs::Float64>("/vel_boxX_groundTruth", queue_size);
		m_pub_vel_laserx_kf = m_node->advertise<std_msgs::Float64>("/vel_x_kf", queue_size);
		m_pub_vel_rmse = m_node->advertise<std_msgs::Float64>("/kf_vel_rmse", queue_size);
	}


	void ROS_Logistics::callback_sonar_gz(ConstSonarPtr& msg)
	{
//		cout << "Sonar: " << msg->DebugString() << endl;
//		cout << "Sonar: " << msg->has_contact() << endl;
		cout << "Sonar: " << msg->range_max() << endl;
	}


	/* Establish necessary services, clients, publisher, listener communication */
	void ROS_Logistics::setUp_Gazebo_Communication()
	{
		// Load gazebo
		gazebo::client::setup();

		// Create our node for communication
		m_gazebo_node.reset(new gazebo::transport::Node());
		m_gazebo_node->Init();

		// Listen to Gazebo world_stats topic
		string topic_sonar = "/gazebo/default/turtlebot3_waffle/base_footprint/ultrasonic/sonar";
		m_sub_sonar_gz = m_gazebo_node->Subscribe(topic_sonar, &ROS_Logistics::callback_sonar_gz, this);

	}

}    	// namespace turtlebot3
