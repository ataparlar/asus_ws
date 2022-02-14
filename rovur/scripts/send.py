#!/usr/bin/env python

import actionlib
import rospy
import nav_msgs.srv as nav_srvs
import mbf_msgs.msg as mbf_msgs
import move_base_msgs.msg as mb_msgs
from geometry_msgs.msg import PoseStamped

class mbf_goal:
    def __init__(self):
        rospy.init_node("move_base_relay")
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.simple_goal_cb)
        self.mbf_mb_ac = actionlib.SimpleActionClient("move_base_flex/move_base", mbf_msgs.MoveBaseAction)
        self.mbf_mb_ac.wait_for_server(rospy.Duration(20))

    def simple_goal_cb(self,msg):
        print("hello")
        self.mbf_mb_ac.send_goal(mbf_msgs.MoveBaseGoal(target_pose=msg))
        rospy.logdebug("Relaying move_base_simple/goal pose to mbf")
        self.mbf_mb_ac.wait_for_result()
        status = self.mbf_mb_ac.get_state()
        result = self.mbf_mb_ac.get_result()
        rospy.logdebug("MBF execution completed with result [%d]: %s", result.outcome, result.message)


    

if __name__ == '__main__':
    mbf_goal()
    rospy.spin()
