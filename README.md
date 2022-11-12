
# ------ CREATE GAZEBO MAP ----- #

Open Gazebo
Edit -> Building Editor
Wall
Draw 10 x 10 box and any internal walls
Save
Exit Building Editor


# ------- CREATE RVIZ MAP ------ #
rosrun map_server map_saver -f ~/sf_simulation


# ---- MOVE TURTLEBOT MODEL ---- #

roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch


# ---- TEST SENSOR OUTPUTS ----- #

rosrun rqt_gui rqt_gui


# ------- SIMULATION SETUP ----- #

source /opt/ros/neotic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

cd ~/src
copy contents of turtlebot folder here


catkin_create_pkg sensor_fusion 
cd ~/sensor_fusion

copy contents of sensor fusion folder here
cd ../..
catkin_make


# ------- FILE LOCATIONS ------- #

catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/sf_world.launch
catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/sf.world
catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/sf_model/model.config
catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/sf_model/model.sdf

A copy of the files is in the simulation folder for easier viewing, but you need to
modify the originals to change the launch files.


Note: python_sim and ros_sim is old code, but it might be a good reference or starting
point, so I included it too.


# ------- RUN SIMULATION ------- #

TAB 1:

roscore


TAB 2:

roslaunch turtlebot3_gazebo sf_world.launch


TAB 3:

roslaunch turtlebot3_slam turtlebot3_slam.launch


TAB 4:

rosrun rviz rviz -d 'rospack find turtlebot3_slam'/rviz/turtlebot3_slam.rviz


TAB 5:

roslaunch turtlebot3_navigation amcl_demo.launch


TAB 6:

rosrun sensor_fusion navigate.py

