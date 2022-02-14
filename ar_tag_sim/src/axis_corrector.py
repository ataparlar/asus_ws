#!/usr/bin/env python 

# Halil Faruk Karagoz 
# At first i wrote this for fixing the axis then it was working correct than we fixed it with static frame 
#After this code wasn't necessary to use but still couldn't delete it bcs it was my first proper ros code <kiyamadim>
import rospy 
import tf2_geometry_msgs
import tf
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from math import pi

class axis_correction:
    def __init__(self):
        self.correct_marker = Marker()
        rospy.init_node("axis_corrector")
        self.listener = tf.TransformListener()
        rospy.Subscriber("/visualization_marker",Marker,self.vis_marker_correct)
        self.pub2 = rospy.Publisher("vis_marker_correct",Marker,queue_size=10)
        self.rate = rospy.Rate(10)
        rospy.spin()

       

    def vis_marker_correct(self,data):
        self.correct_marker = data
        temp = data.pose.position.x
        self.correct_marker.pose.position.x =data.pose.position.z
        self.correct_marker.pose.position.z =temp
        temp = self.correct_marker.pose.position.z
        self.correct_marker.pose.position.z = -self.correct_marker.pose.position.y
        self.correct_marker.pose.position.y = - temp
        
        self.roll,self.pitch,self.yaw = tf.transformations.euler_from_quaternion([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])
        x,y,z,w = tf.transformations.quaternion_from_euler(self.yaw,self.roll + pi/2,-self.pitch)
        self.correct_marker.pose.orientation.x = x
        self.correct_marker.pose.orientation.y = y
        self.correct_marker.pose.orientation.z = z
        self.correct_marker.pose.orientation.w = w
        self.pub2.publish(self.correct_marker)
        self.rate.sleep()
        #print(type(("ar_marker %d",data.id)))
if __name__ == "__main__":
    axis_correction()





    
