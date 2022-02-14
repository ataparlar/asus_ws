#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist,Transform
from math import pi
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf.msg import tfMessage
from math import sin, cos, pi,atan2
import time


class approach:
    def __init__(self):
        rospy.init_node("approach")
        self.t = Twist()
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom)
        rospy.Subscriber("/tf", tfMessage, self.detect_ar_tag)

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        r = rospy.Rate(30)
        #Initial position
        self.x_initial_point = 0
        self.y_initial_point = 0
        #Starting_time
        self.start_time = rospy.Time.now().secs
        #Starting velocities
        self.linear_x = 0.4
        self.angular_z =pi / 4
        #Initialized flag it exist to initialize only once 
        self.initilized = False
        #It is flag for sphiral state draw a sphrial 
        self.sphiral_state = True
        #It is a flag for look around state turn around 360 degree
        self.look_around_state = False
        #It is a flag to enable when you see an ar_tag
        self.wait_for_ar_tag = False

        self.post_no = 4

        self.rotating = 0
        
        self.step = 0
        # how many times tag tf appeared 
        # calculate instant radius of sphiral 
        self.r = 0
        # Keep return speed for sphiral state
        self.return_speed_x = None
        self.return_speed_z = None
        #dictionary for ar_tags
        self.ar_tags = {}
        # it is for counting ar tags
        self.temp_ar_tag = {}

        r.sleep()
        rospy.spin()

    def odom(self, data):
        print(self.yaw)
        if (self.initilized == False): # initial point
            self.x_initial_point = data.pose.pose.position.x
            self.y_initial_point = data.pose.pose.position.y
            self.initilized = True
        # position
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z
        # orientation
        x_angular = data.pose.pose.orientation.x
        y_angular = data.pose.pose.orientation.y
        z_angular = data.pose.pose.orientation.z
        w_angular = data.pose.pose.orientation.w

        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            (x_angular, y_angular, z_angular, w_angular))
        if(self.sphiral_state):
            print("I am in sphiral_state")
            self.draw_sphiral()
        if(self.look_around_state):
            print("I am in look_around")
            self.look_around()
        if(self.wait_for_ar_tag):
            print("I am in wait_for_ar_tag_state")
            
        self.t.linear.x = self.linear_x
        self.t.angular.z = self.angular_z
        self.pub.publish(self.t)

    def find_distance(self,first,last):
        return np.sqrt(np.sum(np.square(np.subtract(first,last))))

    def draw_sphiral(self): # draw sphiral 
        if (self.angular_z > 0):
            self.r = self.linear_x / self.angular_z
        
        if(self.yaw  % pi/3  < 0.1 and self.r < 25):
            self.angular_z = 99 * self.angular_z/100
            self.linear_x = 121 * self.linear_x/120
            if(self.linear_x > 0.8 ):
                self.linear_x = 0.8
            self.step += 1

        # go for look_around_state
        if(self.find_distance([self.x_initial_point,self.y_initial_point],[self.x,self.y]) > 8): # distance may change
            self.sphiral_state = False
            self.look_around_state = True
            self.starting_look_around = (rospy.Time.now().secs)
            self.rotating = self.yaw
            self.return_speed_x = self.linear_x
            self.return_speed_z = self.angular_z

    def detect_ar_tag(self, data):
        transform = data.transforms
        for i in transform: # detect ar_tag
            name = i.child_frame_id
            if "ar_marker_" in name:
                #count how many times it see ar_tag
                if name not in self.temp_ar_tag.keys():
                    self.temp_ar_tag[name] =0
                for key,value in self.temp_ar_tag.items():
                    if key == name:
                        self.temp_ar_tag[name] +=1
                    # if it see a tag more than five than it is real
                    if value > 5:
                        self.ar_tags[key] = i.transform
                self.return_speed_x = self.linear_x
                self.return_speed_z = self.angular_z
                self.linear_x = 0
                self.wait_for_ar_tag = True
                self.start_time = (rospy.Time.now().secs)
                self.angular_z = 0
                self.sphiral_state = False
                self.look_around_state = False
                print(self.temp_ar_tag)
        self.is_it_noise(data)

    def is_it_noise(self,data):
        # if it was noise look around again
            if(((rospy.Time.now().secs) - self.start_time > 2) and self.ar_tags and self.wait_for_ar_tag):
                self.wait_for_ar_tag = False
                self.sphiral_state = False
                self.look_around_state = True
                self.starting_look_around = (rospy.Time.now().secs)
        # if not go to next stage 
            elif(self.ar_tags and self.post_no == 4 and len((self.ar_tags)) == 1):
                self.wait_for_ar_tag = False
                self.sphiral_state = False
                self.look_around_state = False
                self.turn_around_ar_tag(self.ar_tags.values()[0])
        
        


    def look_around(self):
        self.linear_x = 0
        self.angular_z = pi/4
        if((rospy.Time.now().secs != self.starting_look_around) and abs(self.yaw - self.rotating) < 0.05):
            self.sphiral_state = True
            self.look_around_state = False
            self.linear_x = self.return_speed_x 
            self.angular_z = self.return_speed_z
            self.x_initial_point = self.x
            self.y_initial_point = self.y
            


    def turn_around_ar_tag(self,transform):
        print("I am in turn around ar_tag ")
        ar_tag_x = transform.translation.z # in relative position positions are different 
        ar_tag_y = transform.translation.x # in relative position positions are different cause can be simulation check in real world
  
        if ar_tag_y > 0.5:
            self.angular_z = -pi/8
        elif ar_tag_y < -0.5:
           self.angular_z = +pi/8

        else:
            self.look_ar_tag = True
            self.angular_z = 0
            self.draw_circle(r =ar_tag_x )
        
    
    def draw_circle(self,r):
        self.linear_x = 0.5
        self.angular_z = self.linear_x/r




if __name__ == "__main__":
    approach()
