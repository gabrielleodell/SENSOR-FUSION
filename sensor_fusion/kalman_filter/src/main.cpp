/**
 * This module contains the main function
 */

#include "ROSLogistics.hpp"
#include "KalmanFilter.hpp"
#include <memory>
using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtlebot3_kf", ros::init_options::AnonymousName);


	/* When the robot ScaraKinematics is changed, just change the object name here and
	 * the rest of the code needn't be touched */
	shared_ptr<turtlebot3::KalmanFilter> turtlebot3_kf_obj = make_shared<turtlebot3::KalmanFilter>();


	/* This starts the ROS server to serve FK, IK, Jacobian, etc */
	turtlebot3::ROS_Logistics rosObj(turtlebot3_kf_obj);

    return 0;
}

