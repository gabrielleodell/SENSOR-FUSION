
# CREATE GAZEBO MAP

Open Gazebo <br/>
Edit -> Building Editor <br/>
Wall <br/>
Draw 10 x 10 box and any internal walls <br/>
Save <br/>
Exit Building Editor <br/>


# CREATE RVIZ MAP

rosrun map_server map_saver -f ~/sf_simulation


# TELEOPERATE TURTLEBOT

roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch


# TEST SENSOR OUTPUTS

rosrun rqt_gui rqt_gui


# SIMULATION SETUP

source /opt/ros/neotic/setup.bash <br/>
mkdir -p ~/catkin_ws/src <br/>
cd ~/catkin_ws/ <br/>
catkin_make <br/>

cd ~/src <br/>
copy turtlebot3 folders here <br/>

catkin_create_pkg sensor_fusion <br/>
cd ~/sensor_fusion <br/>
copy contents of sensor fusion folder here <br/>

cd ../.. <br/>
catkin_make <br/>


# OBSTACLE PLUGIN SETUP

cd ~/catkin_ws/devel/lib <br/>
sudo cp libCollisionActorPlugin.so /usr/lib/x86_64-linux-gnu/gazebo-11/plugins <br/>
ls <br/>
Check that libCollisionActorPlugin.so appears in list (fine if it's in green) <br/>


# FILE LOCATIONS

Launch: <br/>
catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch/sf_world.launch <br/>

Models: <br/>
catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/animated_box_x <br/>
catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/animated_box_y <br/>
catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/box <br/>
catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/sf_model <br/>

Obstacle Plugin: <br/>
catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/src/CollisionActorPlugin.cc <br/>
catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/src/CollisionActorPlugin.hh <br/>

World: <br/>
catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/sf.world <br/>

A copy of the files is in the simulation folder for easier viewing, but you need to <br/>
modify the originals to change the launch files.

Note: python_sim and ros_sim is old code, but it might be a good reference or starting
point, so I included it too.


# RUN SIMULATION

TAB 1: SETTINGS

source ~/.bashrc <br/>
roscore



TAB 2: LAUNCH SIMULATION

roslaunch turtlebot3_gazebo sf_world.launch



TAB 3: START RVIZ

roslaunch turtlebot3_slam turtlebot3_slam.launch



TAB 4: RUN RVIZ

rosrun rviz rviz -d 'rospack find turtlebot3_slam'/rviz/turtlebot3_slam.rviz



TAB 5: RVIZ AMCL

roslaunch turtlebot3_navigation amcl_demo.launch


TAB 6: SONAR TOPIC

gz topic -e /gazebo/default/turtlebot3_waffle/base_footprint/ultrasonic/sonar

