cmake_minimum_required(VERSION 3.0.2)
project(sensor_fusion)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  rospy
  std_msgs
  nav_msgs
  gazebo_msgs
)

# Generate services in the 'srv' folder
#add_service_files(
#  FILES
#)


# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  gazebo_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES scara_kinematics
#  CATKIN_DEPENDS message_generation message_runtime rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  ros_sim/navigate.py
  ros_sim/nav_to_goal.py
  model_states/boxes.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
