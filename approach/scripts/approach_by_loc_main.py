#!/usr/bin/env python
import rospy
from actionlib_msgs.msg import GoalStatusArray,GoalID
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np 
from fiducial_msgs.msg import FiducialTransformArray
import math as m
from send_goal import send_goal
from std_msgs.msg import Bool,String
from geometry_msgs.msg import Twist
import time


class Approach_by_loc:
    def __init__(self):
        self.pub_e_stop = rospy.Publisher("/e_stop",Bool,queue_size=10)
        self.hand_drive = rospy.Publisher("/cmd_hand_drive",Twist,queue_size=10)

        self.x, self.y = None,None
        self.roll,self.pitch,self.yaw = None,None,None
        self.flag_found_ar_tag = False
        self.flag_draw_sphiral = True
        self.flag_look_around = False
        self.initialized = False
        self.flag_yolo = False
        self.flag_yolo_okey = False # for giving the first goal 
        self.starting_angle = None
        self.status = 1
        self.yolo_output = None
        rospy.Subscriber("/odometry/filtered",Odometry,self.odom)
        while self.x == None:
            rospy.sleep(1)

        self.pts = [] # locations for sphrial
        self.starting_point = (2,0)  # starting point 
        self.start_odom = np.array((self.x,self.y)) # holding starting odometry
        self.goal_number = 0 # goal number 
        self.spihiral_angle =  m.pi/3 # rotated angle in every step 
        self.starting_angle = self.spihiral_angle  # yaw of first goal 
        self.sphiral_points(1.1,1.15,self.spihiral_angle) # found sphiral points since sphiral grows exponentially at the beginning it grows with bigger r then after a point it will use smaller r
        
        self.ar_tags = {} 
        # it is for counting ar tags
        self.count_tag = {}
        # since you rotate according to zero point if you are at 0 you can not rotate so you give a first position
        
        rospy.Subscriber("/fiducial_transforms",FiducialTransformArray,self.detect_ar_tag)
        rospy.Subscriber('/turningway',String,self.yolo)
        self.Rate = rospy.Rate(5)

    
    def yolo(self,data):
        if(len(self.count_tag) == 0):
            self.yolo_output = data.data
            self.timer_for_yolo = rospy.Time.now().secs
            self.flag_yolo = True
            self.flag_draw_sphiral = False
            if(self.yolo_output == "okay") and (self.flag_yolo_okey == False):
                send_goal(2,0,0,"base_link",wait_flag=False)
                self.flag_yolo_okey = True

    def yolo_result(self):
        if(self.yolo_output != None):
            if(self.yolo_output == "okay") and (self.status == 3): # if it is oke 
                send_goal(2,0,0,"base_link",wait_flag=False)
            t = Twist()
            if(self.yolo_output == "left"):
                t.angular.z = m.pi/5
                self.hand_drive.publish(t)
            if(self.yolo_output == "right"):
                t.angular.z = -m.pi/5
                self.hand_drive.publish(t)

        

    def detect_ar_tag(self, data):
        for i in data.transforms:
            name = i.fiducial_id
            if(not self.flag_found_ar_tag):
                self.flag_found_ar_tag = True
                self.flag_draw_sphiral = False
                self.timer_for_tag = rospy.Time.now().to_sec()
                print("I might see a tag just checking!")
                self.stop()
            if name not in self.count_tag.keys():
                self.count_tag[name] =0
            for key,value in self.count_tag.items():
                if key == name:
                    self.count_tag[name] +=1
                if value > 10:
                    self.ar_tags[key] = i.transform

    def run(self):
        while (not rospy.is_shutdown()):
            if self.initialized == False and self.starting_angle != None:
                self.go_on()
                send_goal(self.x + self.starting_point[0],self.y + self.starting_point[1],self.starting_angle,wait_flag=  False)
                self.initialized = True
                rospy.Subscriber("move_base/status",GoalStatusArray,self.move_base_status)
            if (self.status == 3 and  (self.flag_draw_sphiral)) :
                self.draw_sphiral()
            if(self.flag_look_around):
                self.turn_around(m.pi/3)
            if self.flag_yolo:
                self.yolo_result()
                if float(rospy.Time.now().secs) - float(self.timer_for_yolo) > 0.2:
                    self.flag_draw_sphiral = True
                    self.flag_yolo = False
                    self.flag_yolo_okey = False
            if (self.flag_found_ar_tag):
                if(rospy.Time.now().to_sec() -  self.timer_for_tag > 3  and len(self.ar_tags) == 0):
                    self.starting_look_around = rospy.Time.now().secs
                    self.initial_yaw = self.yaw
                    self.flag_found_ar_tag = False
                    self.flag_draw_sphiral = False
                    self.flag_look_around = True
                    self.go_on()
                    self.count_tag.clear()
            if len(self.ar_tags) != 0:
                marker_x,marker_y = list(self.ar_tags.values())[0].translation.z,-list(self.ar_tags.values())[0].translation.x
                self.go_on()
                send_goal(0,0,m.atan2(marker_y,marker_x),"base_link")
                marker_x,marker_y = list(self.ar_tags.values())[0].translation.z,-list(self.ar_tags.values())[0].translation.x
                if(marker_x-5 > 0):
                    send_goal(marker_x-5,0,0,"base_link")
                return True
            self.Rate.sleep()


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
            if(self.find_distance(self.start_odom,last_p) < 10):
                last_p = r2 * np.matmul(rotation_matrix,last_p)
            
            elif self.find_distance(self.start_odom,last_p) > 25:
                break
            else:
                last_p = r1 * np.matmul(rotation_matrix,last_p)
            p = (self.x + last_p[0,0],self.y + last_p[1,0])
            self.pts.append(p)

    def find_distance(self,pt1,pt2):
        return np.sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)

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
        

        

    def move_base_status(self,data):
        self.stat_list = data.status_list
        for i in self.stat_list :
            self.status = i.status

        


    def draw_sphiral(self):
        self.goal_number += 1
        x,y = self.pts[self.goal_number]
        x_n,y_n = self.pts[self.goal_number+1]
        rospy.loginfo("Get a new Goal")
        rospy.loginfo(self.goal_number)
        send_goal(x,y,m.atan2(y_n-self.y,x_n-self.x)+0.5,wait_flag= False)

            

    def turn_around(self,angular_speed):
        t  = Twist()
        t.angular.z = angular_speed
        self.hand_drive.publish(t)

        if((rospy.Time.now().secs -  self.starting_look_around > 2) and abs(self.yaw - self.initial_yaw) < 0.05):
            rospy.loginfo("Okey false alarm go on sphiral")
            self.flag_draw_sphiral = True
            self.flag_look_around = False
    

        


if __name__ == "__main__":
    print(Approach_by_loc())
