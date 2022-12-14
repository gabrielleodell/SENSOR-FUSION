#! usr/bin python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

rospy.init_node('odom_pub', anonymous=True)

odom_pub_x=rospy.Publisher('/box_x', Odometry, queue_size=10)
odom_pub_y=rospy.Publisher('/box_y', Odometry, queue_size=10)

rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv=rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

odom_x=Odometry()
odom_y=Odometry()

header_x = Header()
header_y = Header()

header_x.frame_id='/odom'
header_y.frame_id='/odom'

model_x = GetModelStateRequest()
model_y = GetModelStateRequest()

model_x.model_name='Box_1'
model_y.model_name='Box_2'

r = rospy.Rate(2)

while not rospy.is_shutdown():
    result_x = get_model_srv(model_x)
    result_y = get_model_srv(model_y)

    odom_x.pose.pose = result_x.pose
    odom_y.pose.pose = result_y.pose

    odom_x.twist.twist = result_x.twist
    odom_y.twist.twist = result_y.twist

    header_x.stamp = rospy.Time.now()
    header_y.stamp = rospy.Time.now()

    odom_x.header = header_x
    odom_y.header = header_y

    odom_pub_x.publish(odom_x)
    odom_pub_y.publish(odom_y)

    r.sleep()
