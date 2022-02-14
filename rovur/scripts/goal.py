#!/usr/bin/env python

import actionlib
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class sendGoal():
    def __init__(self):

        rospy.init_node("move_base_publish_goal")

        self.goal_to_pub = PoseStamped()
        self.goal_to_pub.header.frame_id = "odom"
        self.goal_to_pub.header.stamp = rospy.Time.now()
        self.goal_to_pub.pose.position.x = 2
        self.goal_to_pub.pose.position.y = 0
        self.goal_to_pub.pose.orientation.x = 0
        self.goal_to_pub.pose.orientation.y = 0
        self.goal_to_pub.pose.orientation.z = 0
        self.goal_to_pub.pose.orientation.w = 1


        self.pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
        rospy.Subscriber("move_base_simple/yolla", String, self.command_cb)

        rospy.spin()

    def command_cb(self, data):
        print("ehuehu")
        if data.data == "yolla":
            print("anan")
            self.pub.publish(self.goal_to_pub)


if __name__ == '__main__':
    try:
        sendGoal()
    except KeyboardInterrupt:
        rospy.signal_shutdown()
    