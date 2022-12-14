#! usr/bin python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

rospy.init_node('odom_pub', anonymous=True)
r = rospy.Rate(2)

rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv=rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

# Setup x
odom_pub_x=rospy.Publisher('/box_x', Odometry, queue_size=10)
odom_x=Odometry()
header_x = Header()
header_x.frame_id='/odom'
model_x = GetModelStateRequest()
model_x.model_name='Box_1'

# Set up y
odom_pub_y=rospy.Publisher('/box_y', Odometry, queue_size=10)
odom_y=Odometry()
header_y = Header()
header_y.frame_id='/odom'
model_y = GetModelStateRequest()
model_y.model_name='Box_2'


while not rospy.is_shutdown():

    # Run x
    result_x = get_model_srv(model_x)
    odom_x.pose.pose = result_x.pose
    odom_x.twist.twist = result_x.twist
    header_x.stamp = rospy.Time.now()
    odom_x.header = header_x
    odom_pub_x.publish(odom_x)

    # Run y
    result_y = get_model_srv(model_y)
    odom_y.pose.pose = result_y.pose
    odom_y.twist.twist = result_y.twist
    header_y.stamp = rospy.Time.now()
    odom_y.header = header_y
    odom_pub_y.publish(odom_y)

    r.sleep()
