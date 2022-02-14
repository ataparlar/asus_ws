#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos

class listener():
	def __init__(self):
                self.line = np.empty(shape=((1,2)))
                self.yaws = np.empty(shape=((1,2)))
                rospy.init_node('global_plan_listener', anonymous=False)
                self.sub = rospy.Subscriber("/locomove_base/NavfnROS/plan", Path, self.poseCallback)
                self.rate = rospy.Rate(10) #10Hz
                
                while not rospy.is_shutdown():
                        self.rate.sleep()

        def poseCallback(self, data):
                #np.delete(self.line,0,axis=0)
                i = 0
                for pose in data.poses:
                        self.line = np.append(self.line, [[pose.pose.position.x, pose.pose.position.y]], axis=0)
                        new_x = sin(pose.pose.orientation.w)
                        new_y = cos(pose.pose.orientation.w)
                        self.yaws = np.append(self.yaws, [[new_x, new_y]])
                        plt.Arrow(self.line[i,0], self.line[i,1], self.line[i,0]*new_x, self.line[i,1]*new_y)
                        i+=1
                print("lann")
                
                plt.scatter(self.line[:,0], self.line[:,1])
                plt.draw()
                plt.pause(0.0001)
                plt.clf()
                #self.line = np.zeros(shape=((1,2)))
                self.line = np.empty(shape=((1,2)))
                print("hmm")
		
if __name__ == '__main__':
    try:
        listener()
    except KeyboardInterrupt:
        rospy.signal_shutdown("killme")