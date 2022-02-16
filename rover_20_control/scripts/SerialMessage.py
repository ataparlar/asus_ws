#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Author(s): ITU Rover Team '20 Software Sub-team
# Edited by Basan 
# 24/12/2021

import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float64MultiArray
import rospy

#TODO: serial comm test bit

class SerialMessage():

	def __init__(self):

		self.joint_positions = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
		self.joint_velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
		self.arm_mode = 8
		self.string_list = ["0000","0000","0000","0000","0000","0000","0000"]
		self.activity_indicator = 0
		self.arm_indicator = 0
		self.activity_indicator_final = 0
		self.probe_servo = 0
		self.last_movement = 0
		self.left_wheel = 0
		self.right_wheel = 0

		self.twist_var = Twist()
		self.wheel_velocities = ["0000","0000"]

		self.right_wheel = 0
		self.left_wheel = 0
		self.cmd_timer = rospy.Time.now()
		self.comm_check = "0"
		rospy.Subscriber('/nav_vel',Twist,self.autonomous_cb)
		rospy.Subscriber('/joy_teleop/cmd_vel',Twist,self.teleop_cb)
		rospy.Subscriber('/joint_states/send',Float64MultiArray,self.joint_cb)
		#rospy.Subscriber('/cmd_vel',Twist, self.twist_cb)

	def autonomous_cb(self,data):
		self.activity_indicator = 3
		self.last_movement = rospy.Time.now().to_sec()
		
	def teleop_cb(self,data):
		if not (data.linear.x == 0 and data.linear.y == 0 and data.linear.z == 0 and data.angular.x == 0 and data.angular.y == 0 and data.angular.z == 0):
			self.activity_indicator = 2
			self.last_movement = rospy.Time.now().to_sec()

	def joint_cb(self,data):
		self.arm_indicator = data.data[8]

	def wheelParser(self, data):
		if(data != 0):
			data = max(80, min(abs(data), 180)) #clamp data between 80-180
		wheel_string = str(abs(data)).zfill(3)
		return wheel_string
		
	def wheelSign(self, data):
		if(data < 0):
			way = 1
			data *= -1
		else:
			way = 0
		return str(way)

	#def twist_cb(self, data):
	#	self.cmd_timer = rospy.Time.now().to_sec()
	#	print(self.cmd_timer)
	
	def wheelVelToString(self):

		if self.twist_var.linear.x >= 0:
			self.left_wheel = self.twist_var.linear.x - self.twist_var.angular.z
			self.right_wheel = self.twist_var.linear.x + self.twist_var.angular.z
		elif self.twist_var.linear.x < 0:
			self.left_wheel = self.twist_var.linear.x + self.twist_var.angular.z
			self.right_wheel = self.twist_var.linear.x - self.twist_var.angular.z
		
		#if rospy.Time.now().to_sec() - self.cmd_timer > 0.5:
		#	self.left_wheel = 0
		#	self.right_wheel = 0
		self.wheel_velocities[0] =  self.wheelSign(int(self.left_wheel)) + self.wheelParser(int(self.left_wheel))
		self.wheel_velocities[1] =  self.wheelSign(int(self.right_wheel)) + self.wheelParser(int(self.right_wheel))

	def jointAngleToString(self):

		print("Joint angle to string")
		for i in range(len(self.joint_positions)):
			radian = self.joint_positions[i]

			degrees = 180*radian/math.pi
		
			if degrees > 0:
				string_angle = "1"
			elif degrees <= 0:
				string_angle = "0"
			degrees = abs(degrees)
			if degrees > 180:
				degrees = 180

			mapped_degree = str(int(degrees*5))
			string_angle = string_angle + "0"*(3-len(mapped_degree)) + mapped_degree
			self.string_list[i] = string_angle
		
	def jointVelocityToString(self):
		
		for i in range(len(self.joint_velocities)):

			velocity = self.joint_velocities[i]
			if velocity < 0:
				direction = "0"
			else:
				direction = "1"
			velocity = abs(velocity*1.25)
			string_vel = str(int(velocity))
			string_vel = direction + "0"*(3-len(string_vel)) + string_vel
			self.string_list[i] = string_vel

	def returnFinalMsg(self):

		final_msg = "S"		
		final_msg += self.wheel_velocities[0] + self.wheel_velocities[1]

		for i in self.string_list:
			final_msg += i
			
		time = rospy.Time.now().to_sec()
		if time - self.last_movement > 5:
			self.activity_indicator = 1

		self.activity_indicator_final = max(self.activity_indicator,self.arm_indicator)
		final_msg += 16*"0" + str(int(self.arm_mode)) + str(int(self.activity_indicator_final))+ str(int(self.probe_servo))+"F"
		
		return final_msg 
