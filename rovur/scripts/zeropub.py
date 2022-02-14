#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Author(s): ITU Rover Team '20 Software Sub-team
# Edited by Basan 

import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float64MultiArray
from mbf_msgs.msg import ExePathActionResult
from geometry_msgs.msg import Vector3
import rospy

class zeroPub():
    def __init__(self):

        rospy.init_node("move_base_publish_zero")

        self.pub = rospy.Publisher("/zero_vel", Twist, queue_size=10)
        rospy.Subscriber("/move_base_flex/exe_path/result", ExePathActionResult, self.listener_cb)

        self.zero_vel = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))

        rospy.spin()

    def listener_cb(self, data):
        if(data.status.status == 3 and data.status.text == "Controller succeeded; arrived at goal!"):
            self.pub.publish(self.zero_vel)

if __name__ == '__main__':
    try:
        zeroPub()
    except KeyboardInterrupt:
        rospy.signal_shutdown()