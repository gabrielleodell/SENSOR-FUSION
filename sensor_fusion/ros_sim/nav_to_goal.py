#! usr/bin python

# TurtleBot must have minimal.launch & amcl_demo.launch running prior to starting this script
# For simulation: launch gazebo world & amcl_demo prior to run this script

import os
import rospy
import actionlib

from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class GoToPose():
    def __init__(self):

        self.goal_sent = False

        # Handle shutdown
        rospy.on_shutdown(self.shutdown)

        # Setup client/server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Connecting to action server")
        connected = self.move_base.wait_for_server(rospy.Duration(5))

        if not connected:
            rospy.loginfo("Action server not available")
            return

        rospy.loginfo("Connected to action server")

    def goto(self, pos, quat):
        self.goal_sent = True

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], pos['z']),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        self.move_base.send_goal(goal)

        success = self.move_base.wait_for_result(rospy.Duration(0.5)) 
        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()

        rospy.loginfo("Stop")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('navigation_py', anonymous=False)
        navigator = GoToPose()

        position = {'x': 8.5, 'y' : -8.5, 'z' : 0.000}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        rospy.loginfo("Moving to (%s, %s)", position['x'], position['y'])
        success = navigator.goto(position, quaternion)

        if success:
            rospy.loginfo("Goalreached")
        else:
            rospy.loginfo("Robot failed to reach goal")

        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
