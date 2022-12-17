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


# CHANGE PATH

In sf.world, there are five absolute paths for the TurtleBot model. Replace with <br/>
your path to turtlebot3_description. Located in Lines 257, 272, 281, 511, 593.


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


# RUN SIMULATION

TAB 1: SETTINGS

source ~/.bashrc <br/>
roscore


TAB 2: LAUNCH SIMULATION

roslaunch turtlebot3_gazebo sf_world.launch


TAB 3: TELEOPERATE TURTLEBOT

roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch


TAB 3: KALMAN FILTER

rosrun turtlebot3_sensorFusion turtlebot3_kf


TAB 4: SONAR TOPIC

rostopic echo /sonae


TAB 5: MOVING BOX X TOPIC

rostopic echo /box_x


TAB 6: MOVING BOX Y TOPIC

rostopic echo /box_y
