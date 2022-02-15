#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from math import pi, sqrt
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
from locomove_base.srv import *


class oscillation_handler:
    def __init__(self):
        rospy.init_node('cmd_vel_manipulator', anonymous=False)

        self.YAW_GOAL_TOLERANCE = rospy.get_param("/locomove_base/DWBLocalPlanner/yaw_goal_tolerance")
        self.XY_GOAL_TOLERANCE = rospy.get_param("/locomove_base/DWBLocalPlanner/xy_goal_tolerance")

        # Goal variables
        self.goal_x = Float64().data
        self.goal_y = Float64().data
        self.goal = PoseStamped()

        # Odometry variables
        self.x = Odometry().pose.pose.position.x
        self.y = Odometry().pose.pose.position.y
        self.yaw = 0

        self.theta_velocity_buffer = []
        self.oscillation_flag = False
        self.oscillation_mode = 8
        self.oscillation_last_element = 1
        self.manipulator_twist = Twist()
        self.control_array = []

        # Twist Variables 
        self.manipulator_twist = Twist()
        self.zero_twist = Twist()
        self.zero_twist.linear.x = 0.0
        self.zero_twist.linear.y = 0.0
        self.zero_twist.angular.z = 0.0

        self.goal_yaw = None

        self.first_yaw = None

        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb)
        self.cmd_sub = rospy.Subscriber("/rover_velocity_controller/cmd_vel", Twist, self.cmd_callback)
        self.odom_sub = rospy.Subscriber("/odometry/filtered",Odometry, self.odom_cb)
        #self.cmd_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback)
        self.cmd_pub = rospy.Publisher("/handler_vel", Twist, queue_size=1)
        self.stop = rospy.ServiceProxy('/locomove_base/stopper_service', osci)
        self.pose_sub = rospy.Subscriber("/locomove_base/DWBLocalPlanner/global_plan", Path, self.pose_cb)
        self.rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if(self.oscillation_flag == True): # If rover is in oscillation

                direction = self.find_direction(self.yaw, self.goal_yaw)
                if(direction == "right"):
                    self.manipulator_twist.angular.z = -1.0
                else:
                    self.manipulator_twist.angular.z = 1.0

                if self.pisagor(self.goal_y - self.y,self.goal_x - self.x) < self.XY_GOAL_TOLERANCE + 0.1: # If rover is in XY tolerance
                    while True:
                        if abs(self.yawMap(self.yaw ,self.goal_yaw)) < self.YAW_GOAL_TOLERANCE + 0.1: 
                            print("ziro")
                            self.stop(1)
                            self.oscillation_flag = False
                            break
                        else:# If rover is not in the yaw tolerance, publish constant velocity to turn to the right direction. 
                            self.cmd_pub.publish(self.manipulator_twist) 
                        self.rate.sleep()
                else:
                    while (abs(self.yawMap(self.yaw ,self.first_yaw)) > self.YAW_GOAL_TOLERANCE + 0.1):
                        self.cmd_pub.publish(self.manipulator_twist)
                    self.manipulator_twist.angular.z = 0
                    self.manipulator_twist.linear.x = 0.5
                    for i in range(0,20):
                        self.cmd_pub.publish(self.manipulator_twist)
                    self.manipulator_twist.linear.x = 0
                    self.rate.sleep()
            self.rate.sleep()
        

    def double_equal(self, a, b, epsilon=0.001):
        if(a != 0 and b != 0):
            return abs(a-b) < epsilon
        else:
            return False
        
    def cmd_callback(self, data):

        #oscillation detector

        if(len(self.theta_velocity_buffer) < 8):
            self.theta_velocity_buffer.append(data.angular.z)
        else:
            self.theta_velocity_buffer.pop(0)
            self.theta_velocity_buffer.append(data.angular.z)
    
            a = self.theta_velocity_buffer[3]
            b = self.theta_velocity_buffer[4]

            if((self.theta_velocity_buffer[0] == self.theta_velocity_buffer[1]) and (self.theta_velocity_buffer[1] == self.theta_velocity_buffer[2])
                and (self.theta_velocity_buffer[2] == self.theta_velocity_buffer[3]) and (self.theta_velocity_buffer[4] == self.theta_velocity_buffer[5]) 
                and (self.theta_velocity_buffer[5] == self.theta_velocity_buffer[6]) and (self.theta_velocity_buffer[6] == self.theta_velocity_buffer[7])
                and self.double_equal(a,(-1)*b)):
                rospy.logwarn("Titriyoz haa 8")
                self.oscillation_flag = True
                self.oscillation_mode = 8
                for i in range(20):
                    self.rate.sleep()
                self.oscillation_last_element = self.theta_velocity_buffer[7]

            elif((self.theta_velocity_buffer[1] == self.theta_velocity_buffer[2]) and (self.theta_velocity_buffer[2] == self.theta_velocity_buffer[3])
                and (self.theta_velocity_buffer[4] == self.theta_velocity_buffer[5]) and (self.theta_velocity_buffer[5] == self.theta_velocity_buffer[6])
                and self.double_equal(a,(-1)*b)):
                rospy.logwarn("Titriyoz haa 6")
                self.oscillation_flag = True
                self.oscillation_mode = 6
                for i in range(20):
                    self.rate.sleep()
            else:
                self.oscillation_flag = False

    def goal_cb(self,data): # Goal callback function to get goal data such as yaw angle, x and y positions
        self.goal = data
        self.goal_yaw = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])[2]
        self.goal_x = data.pose.position.x
        self.goal_y = data.pose.position.y
        
    
    def odom_cb(self,data): 
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.yaw = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2]

    def pose_cb(self, data):
        self.first_yaw = euler_from_quaternion([data.poses[1].pose.orientation.x, data.poses[1].pose.orientation.y, data.poses[1].pose.orientation.z, data.poses[1].pose.orientation.w])[2]

    def pisagor(self, x, y): # To calculate pisagor of 2 given numbers
        return sqrt(x**2 + y**2)

    def find_direction(self, current_yaw, goal_yaw):

        #clamp between 0, 2pi
        current_yaw += 2*pi
        goal_yaw += 2*pi

        difference = goal_yaw - current_yaw

        if(current_yaw < goal_yaw and abs(difference) <= pi):
            return "left"
        elif(current_yaw < goal_yaw and abs(difference) > pi):
            return "right"
        elif(current_yaw > goal_yaw and abs(difference) <= pi):
            return "right"
        else:
            return "left"

        
    def yawMap(self, current_yaw, goal_yaw): # To choose small arc (yaw difference)
        return (goal_yaw - current_yaw)


if __name__ == '__main__':
    try:
        oscillation_handler()
    except KeyboardInterrupt:
        rospy.signal_shutdown("Exception thrown")
