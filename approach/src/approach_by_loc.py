#!/usr/bin/env python
# client

from __future__ import print_function
import rospy
from move_base_msgs.msg import MoveBaseGoal
from approach.srv import *

class Approach_by_loc:
    def __init__(self):
        rospy.init_node("approach_by_loc")
        self.draw_sphiral()


    def draw_sphiral(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = -10
        goal.target_pose.pose.orientation.w = 1.0
        
        send_goal = rospy.ServiceProxy("send_goal",SendGoal)
        send_goal(goal)
        


if __name__ == "__main__":
    Approach_by_loc()