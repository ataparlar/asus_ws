#!/usr/bin/env python

import rospy
import time
from tf.msg import tfMessage
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from math import atan2,pi
from visualization_msgs.msg import Marker
from tf.msg import tfMessage

def odom(data):

   
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z

    x_angular = data.pose.pose.orientation.x
    y_angular = data.pose.pose.orientation.y
    z_angular = data.pose.pose.orientation.z
    w_angular = data.pose.pose.orientation.w

    roll, pitch, yaw = euler_from_quaternion(
        (x_angular, y_angular, z_angular, w_angular))

# def marker(data):
#     transform = data.transforms
#     static_frame = None
#     for i in transform: # detect ar_tag
#         name = i.child_frame_id
#         if "correct_camera_link" == name:
#             static_frame = i
#     for i in transform: # detect ar_tag
#         name = i.child_frame_id
#         if "ar_marker_" in name:
#             if static_frame :
#                print(static_frame.invers_Times(i))
    
def marker(data):

    marker = data
    roll,pitch,yaw = euler_from_quaternion((marker.pose.orientation.x,marker.pose.orientation.y,marker.pose.orientation.z,marker.pose.orientation.w))
    print(roll,pitch,yaw)
   

if __name__ == "__main__":
    
    rospy.init_node("temp")

    rospy.Subscriber("/odometry/filtered", Odometry, odom)
    rospy.Subscriber("/visualization_marker", Marker, marker)
    rospy.spin()
    
    
