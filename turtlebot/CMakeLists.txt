cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_sensorFusion)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Debug")		# change it to "Release" when needed

################################################
## Find all ROS dependencies ##
################################################

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    message_generation
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    tf2_ros
    controller_interface
    hardware_interface
    pluginlib
)

find_package(gazebo)

################################################
## Declare ROS messages, services and actions ##
################################################

#add_service_files(
#  FILES
#  scara_fk.srv
#  scara_ik.srv
#)

#add_message_files(
#  FILES
#  Float64Array.msg
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#  sensor_msgs	
#)

###################################
## Catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS 
    LIBRARIES 
    CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs message_runtime 
    DEPENDS
)

###########
## Build ##
###########

include_directories(
	src/common_utils/include
    src/common_utils/include/eigen
    src/kalman_filter/include
    /usr/include/python3.8
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

########## Building common_utils.so ##########
set(SOURCES_COMMON_UTILS
		src/common_utils/src/Utils.cpp
)
add_library(common_utils ${SOURCES_COMMON_UTILS})
target_link_libraries(common_utils ${catkin_LIBRARIES})


########## Building kinematics node exe ##########
#file(GLOB SOURCES "scara_ros/src/*.cpp")
set(SOURCES_KF 
		src/kalman_filter/src/main.cpp
		src/kalman_filter/src/ROSLogistics.cpp
		src/kalman_filter/src/KalmanFilter.cpp	
)
add_executable(turtlebot3_kf ${SOURCES_KF})
#add_dependencies(turtlebot3_kf visual_servoing_scara_generate_messages_cpp)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
target_link_libraries(turtlebot3_kf ${catkin_LIBRARIES} python3.8 common_utils ${GAZEBO_LIBRARIES} pthread)


########## Building my_vel_controller plugin .so ##########
#set(SOURCES_PLUGIN_MY_VEL_CONTROLLER
#		scara_ros/plugin_myVelController/src/MyVelocityController.cpp
#)
#add_library(my_velocity_controller ${SOURCES_PLUGIN_MY_VEL_CONTROLLER})
#target_link_libraries(my_velocity_controller ${catkin_LIBRARIES} common_utils)


########## Building visual servoing node exe ##########
#set(SOURCES_VISUAL_SERVOING_OPENCV
#		scara_ros/visual_servoing_opencv/src/main.cpp
#		scara_ros/visual_servoing_opencv/src/ROSLogistics_VisualServoing.cpp
#		scara_ros/visual_servoing_opencv/src/VisualServoing.cpp	
#)
#add_executable(visual_servoing ${SOURCES_VISUAL_SERVOING_OPENCV})
#target_link_libraries(visual_servoing ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} common_utils)


#############
## Testing ##
#############

#############
## Install ##
#############
