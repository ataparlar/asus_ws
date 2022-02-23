#!/usr/bin/env python
#Author: Omer Bera Dinc

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion, Point, Pose 
from visualization_msgs.msg import Marker
from send_goal import send_goal 
import math as m 
import numpy as np 


class TurningAround:
    def __init__(self):
        rospy.init_node("turning_node")
        #We subscribed to receive data from AR Tag and Odometry.
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        rospy.Subscriber("/visualization_marker", Marker, self.marker)

        self.orientation = Quaternion() 
        self.position = Point()
        self.ar_orientation = Quaternion()
        self.ar_position = Point()
        self.markers = {}
        self.euler = (0,0,0)
        
        rospy.spin()

    def odom_cb(self, data):
        #rospy.loginfo(data.pose.pose.orientation)

        #Initial orientation data of rover (w.r.t. starting orientation [x:0 y:0 z:0 w:1])
        self.orientation = data.pose.pose.orientation
        #Initial location data of rover 
        self.position = data.pose.pose.position
        
        #We convert the orientation data we received from the quaternion form to the euler form with this line, but it is not necessary to set the rotation.
        self.euler = euler_from_quaternion([self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])

        #print("euler:", self.euler)
    def marker(self,data):
        self.correct_pose = Pose()
        pose = data.pose
        self.correct_pose.position.x = pose.position.z
        self.correct_pose.position.y = - pose.position.x
        print("marker : " + str(self.correct_pose.position.x)  + "  " + str(self.correct_pose.position.y))    
        dist_from_vehicle = self.find_distance((0,0),(self.correct_pose.position.x,self.correct_pose.position.y))  
        #print(dist_from_vehicle)
        beta = m.pi -  m.atan2(self.correct_pose.position.x,self.correct_pose.position.y)
        #print(beta)
        #print(self.yaw)
        alpha = self.euler[2] -  (m.pi/2  - beta)
        x_rel = (dist_from_vehicle + 0.2 ) * m.cos(alpha)
        y_rel = (dist_from_vehicle + 0.2) * m.sin(alpha)
        #print("marker : " + str( x_rel)  + "  " + str(y_rel))
        if(self.position.x != None):
            self.correct_pose.position.x = x_rel + self.position.x
            self.correct_pose.position.y = self.position.y +  y_rel
            #print("marker last : " + str( pose.position.x)  + "  " + str(pose.position.y))
            self.markers[data.id] = self.correct_pose   
        self.turn()

    def find_distance(self,pt1,pt2):
        return np.sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)

  
    def turn(self):
        x1, y1 = self.position.x, self.position.y   #rover
        x2, y2 =  list(self.markers.values())[0].position.x,list(self.markers.values())[0].position.y  #artag
        #print(x2,y2)
        angle = m.atan2(y2-y1,x2-x1)
        
        #calculating x, y points to give the best goal
        extracted_x = 2.4 * m.cos(angle)   #2 comes from 2 meters, changable parameter within 0 and 2, we need to decrease it but it may intersect area of obstacle since movebase inflates them.
        extracted_y = 2.4 * m.sin(angle)

        final_x = x2 - extracted_x
        final_y = y2 - extracted_y

        send_goal(final_x, final_y, angle)

if __name__ == "__main__":
    try:
        TurningAround()

    except KeyboardInterrupt:
        rospy.signal_shutdown()

    