#!/usr/bin/env python
from logging import Handler
import rospy
from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np 
from visualization_msgs.msg import Marker
import math as m
from send_goal import send_goal
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import time


class Approach_by_loc:
    def __init__(self):
        rospy.init_node("approach_by_loc")
        self.pub_e_stop = rospy.Publisher("/e_stop",Bool,queue_size=10)
        self.hand_drive = rospy.Publisher("/cmd_hand_drive",Twist,queue_size=10)

        self.x , self.y = None,None
        self.roll,self.pitch,self.yaw = None,None,None
        self.flag_found_ar_tag = False
        self.flag_draw_sphiral = True
        self.flag_look_around = False

        rospy.Subscriber("/odometry/filtered",Odometry,self.odom)
        time.sleep(1) # wait for odom data

        self.pts = []
        self.starting_point = (2,2) 
        self.start_odom = np.array((self.x,self.y))
        print(self.starting_point)
        self.goal_number = 0
        self.spihiral_angle =  m.pi/3
        self.starting_angle = self.spihiral_angle * 2 + m.pi/2
        self.sphiral_points(1.05,1.08,self.spihiral_angle)

        self.ar_tags = {}
        # it is for counting ar tags
        self.count_tag = {}
         # since you rotate according to zero point if you are at 0 you can not rotate so you give a first position
        

        
        rospy.Subscriber("move_base/status",GoalStatusArray,self.move_base_status)
        rospy.Subscriber("/visualization_marker",Marker,self.detect_ar_tag)
        send_goal(self.x + self.starting_point[0],self.y + self.starting_point[1],self.starting_angle)
        rospy.spin()


    def detect_ar_tag(self, data):
        name = data.id
        rospy.loginfo("Saw_marker but i am not sure")
        if(not self.flag_found_ar_tag):
            self.flag_found_ar_tag = True
            self.flag_draw_sphiral = False
            self.timer_for_tag = rospy.Time.now().to_sec()
            self.stop()
        if name not in self.count_tag.keys():
            self.count_tag[name] =0
        for key,value in self.count_tag.items():
            if key == name:
                self.count_tag[name] +=1
            if value > 20:
                self.ar_tags[key] = data.pose
            print(self.ar_tags)




    def stop(self):
        rospy.loginfo("I stopped")
        for i in range(50):
            self.pub_e_stop.publish(True)
        
    def go_on(self):
        rospy.loginfo("I go on")
        for i in range(50):
            self.pub_e_stop.publish(False)


    def sphiral_points(self,r1,r2,angle):
        rotation_matrix = np.array(([m.cos(angle),-m.sin(angle)],[m.sin(angle),m.cos(angle)]))
        last_p = np.array(([self.starting_point[0]],[self.starting_point[1]]))
        while(True):
            if(np.sqrt(np.sum(np.square(np.subtract(last_p,self.start_odom)))) < 10):
                last_p = r2 * np.matmul(rotation_matrix,last_p)
            
            elif np.sqrt(np.sum(np.square( np.subtract(last_p,self.start_odom)))) > 25:
                break
            else:
                last_p = r1 * np.matmul(rotation_matrix,last_p)
            p = (self.x + last_p[0,0],self.y + last_p[1,0])
            self.pts.append(p)


    def odom(self,data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        orientation = data.pose.pose.orientation
        x_angular = orientation.x
        y_angular = orientation.y
        z_angular = orientation.z
        w_angular = orientation.w

        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            (x_angular, y_angular, z_angular, w_angular))
        if(self.flag_look_around):
            self.turn_around(m.pi/3)
        if (self.flag_found_ar_tag):
            if(rospy.Time.now().to_sec() -  self.timer_for_tag > 5  and len(self.ar_tags) == 0):
                self.starting_look_around = rospy.Time.now().secs
                self.initial_yaw = self.yaw
                print(self.initial_yaw)
                self.flag_found_ar_tag = False
                self.flag_draw_sphiral = False
                self.flag_look_around = True
                self.go_on()
                self.count_tag.clear()

        

    def move_base_status(self,data):
        self.stat_list = data.status_list
        
        for i in self.stat_list :
            self.status = i.status
            self.last_goal_id = i.goal_id
            self.draw_sphiral()
        


    def draw_sphiral(self):
        if (self.status == 3 and  (self.flag_draw_sphiral)) :
            self.goal_number += 1
            x,y = self.pts[self.goal_number]
            rospy.loginfo("Get a new Goal")
            rospy.loginfo(self.goal_number)
            send_goal(x,y,self.starting_angle + self.spihiral_angle * (self.goal_number))

            

    def turn_around(self,angular_speed):
        t  = Twist()
        t.angular.z = angular_speed
        self.hand_drive.publish(t)

        
        if((rospy.Time.now().secs -  self.starting_look_around > 2) and abs(self.yaw - self.initial_yaw) < 0.05):
            rospy.loginfo("Okey false alarm go on sphiral")
            self.flag_draw_sphiral = True
            self.flag_look_around = False
    

        


if __name__ == "__main__":
    k = Approach_by_loc()
   
