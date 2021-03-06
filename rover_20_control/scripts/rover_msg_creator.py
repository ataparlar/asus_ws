#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Author(s): ITU Rover Team '20 Software Sub-team
# Edited 
# 24/12/2021

import rospy
from std_msgs.msg import String, Int16
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from SerialMessage import *


serial_obj = SerialMessage()
serial_obj.activity_indicator_final = 3 

pub = rospy.Publisher("/rover_serial_topic", String, queue_size=10)

ext_comm_check = 0

class CommCheck():
    def __init__(self):
        self.ext_comm_check = 0
        rospy.init_node('message_creator')
        rospy.Subscriber("joint_states/send",Float64MultiArray,self.jointCallback)
        rospy.Subscriber("/joy_teleop/cmd_vel",Twist,self.wheelCallback)
        rospy.Subscriber("/cmd_vel",Twist,self.wheelCallback)
        rospy.Subscriber("/comm_check",Int16,self.extCheckCb)
        rospy.Subscriber('/odometry/wheel',Odometry,self.main)

        rospy.spin()

    def extCheckCb(self,data):
        self.ext_comm_check = rospy.Time.now().to_sec()

    def jointCallback(self,data):
        
        mode_bit = data.data[-3]

        serial_obj.arm_mode = mode_bit
        serial_obj.activity_indicator = data.data[-2]
        serial_obj.probe_servo = data.data[-1]
        if mode_bit == 8:
            serial_obj.joint_positions = data.data[0:7]
        elif mode_bit == 5:
            serial_obj.joint_velocities = data.data[0:7]

    def wheelCallback(self,data):
        multiplier = 120
        
        serial_obj.twist_var.linear.x = data.linear.x * multiplier
        serial_obj.twist_var.angular.z = data.angular.z * multiplier	

    def main(self,data):

        if serial_obj.arm_mode == 5:
            serial_obj.jointVelocityToString()
            serial_obj.wheelVelToString()
        elif serial_obj.arm_mode == 8:
            serial_obj.jointAngleToString()
            serial_obj.wheelVelToString()
        elif serial_obj.arm_mode == 1:
            serial_obj.wheelVelToString()
                
        final_message = serial_obj.returnFinalMsg()
        pub.publish(final_message)             
        print(final_message)
if __name__ == '__main__':
    CommCheck()


