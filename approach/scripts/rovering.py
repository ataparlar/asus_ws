#!/usr/bin/env python
#Author: Omer Bera Dinc

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion, Point, Transform
from fiducial_msgs.msg import FiducialTransformArray
from send_goal import send_goal 
import math as m 
import numpy as np 


class TurningAround:
    def __init__(self):
        self.markers = {}
        self.count_marker = {}
        self.orientation = Quaternion() 
        self.position = Point()
        self.euler = (0,0,0)
        self.starting_time = rospy.Time.now().to_sec()
        self.done = False
        #We subscribed to receive data from ARUCO Tag and Odometry.
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.marker)
        self.Rate = rospy.Rate(1)

    def odom_cb(self, data):
        #rospy.loginfo(data.pose.pose.orientation)

        #Initial orientation data of rover (w.r.t. starting orientation [x:0 y:0 z:0 w:1])
        self.orientation = data.pose.pose.orientation
        #Initial location data of rover 
        self.position = data.pose.pose.position
        
        #We convert the orientation data we received from the quaternion form to the euler form with this line, but it is not necessary to set the rotation.
        self.roll,self.pitch,self.yaw = euler_from_quaternion([self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])

        #print("euler:", self.euler)
    def run(self):
        while not rospy.is_shutdown():
            if (rospy.Time.now().to_sec() - self.starting_time > 6) and (len(self.markers) == 0 or len(self.markers) > 1):
                return False 
            if len(self.markers) == 1 and (not self.done):
                self.done = True
                self.turn()
                return True
            self.Rate.sleep()
            
    def marker(self,data):
        for i in data.transforms:
            self.correct_transform = Transform()
            transform = i.transform
            self.correct_transform.translation.x = transform.translation.z
            self.correct_transform.translation.y = - transform.translation.x
            self.correct_transform.rotation = transform.rotation
            if i.fiducial_id not in self.count_marker.keys():
                self.count_marker[i.fiducial_id] =0
                send_goal(0,0,m.atan2(self.correct_transform.translation.y,self.correct_transform.translation.x),"base_link")
                print("I might see a tag just checking!")
                rospy.sleep(4)
            for key,value in self.count_marker.items():
                if key == i.fiducial_id:
                    self.count_marker[i.fiducial_id] +=1
                if value > 10:
                    self.correct_transform.translation.x += 0.5 # since camera is 50 cm in front of base link
                    #print("marker : " + str(pose.translation.x)  + "  " + str(pose.translation.y))    
                    dist_from_vehicle = self.find_distance((0,0),(self.correct_transform.translation.x,self.correct_transform.translation.y))  
                    #print(dist_from_vehicle)
                    beta = m.pi -  m.atan2(self.correct_transform.translation.x,self.correct_transform.translation.y)
                    #print(beta)
                    #print(self.yaw)
                    alpha = self.yaw -  (m.pi/2  - beta)
                    #print(alpha)
                    x_rel = (dist_from_vehicle ) * m.cos(alpha)
                    y_rel = (dist_from_vehicle ) * m.sin(alpha)
                    #print("marker : " + str( x_rel)  + "  " + str(y_rel))
                    if(self.position.x != None):
                        self.correct_transform.translation.x = x_rel + self.position.x
                        self.correct_transform.translation.y = y_rel + self.position.y 
                        #print("marker last : " + str( pose.translation.x)  + "  " + str(pose.translation.y))
                        self.markers[i.fiducial_id] = self.correct_transform
                    break
        

    def find_distance(self,pt1,pt2):
        return np.sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)

  
    def turn(self):
        x1, y1 = self.position.x, self.position.y   #rover
        x2, y2 =  list(self.markers.values())[0].translation.x,list(self.markers.values())[0].translation.y  #artag
        #print(x2,y2)
        angle = m.atan2(y2-y1,x2-x1)
        
        #calculating x, y points to give the best goal
        extracted_x = 2.4 * m.cos(angle)   #2 comes from 2 meters, changable parameter within 0 and 2, we need to decrease it but it may intersect area of obstacle since movebase inflates them.
        extracted_y = 2.4 * m.sin(angle)

        pt1 = (x2 - extracted_x,y2 - extracted_y)
        pt2 = (x2 + extracted_x,y2 + extracted_y)

        dist1 = self.find_distance((self.position.x,self.position.y),pt1)
        dist2 = self.find_distance((self.position.x,self.position.y),pt2)

        if dist1 < dist2:
            final_x,final_y = pt1
        else:
            final_x,final_y = pt2


        send_goal(final_x, final_y, angle)

if __name__ == "__main__":
    try:
        TurningAround()

    except KeyboardInterrupt:
        rospy.signal_shutdown()

    