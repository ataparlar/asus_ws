#!/usr/bin/env python 

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist



class e_stop:
    def __init__(self):
        rospy.init_node("e_stop")
        self.publisher = rospy.Publisher("/stop",Twist,queue_size=10)
        rospy.Subscriber("/e_stop",Bool,self.callback)
        self.t = Twist()
        self.t.linear.x,self.t.linear.y,self.t.linear.z = (0,0,0)
        self.t.angular.x,self.t.angular.y,self.t.angular.z = (0,0,0)
        self.stop = False

    def callback(self,data):
        if data.data == True:
            self.stop = True
        else:
            self.stop = False

    def pub(self):
        if self.stop :
            self.publisher.publish(self.t)


if __name__ == "__main__":
    e_stop = e_stop()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        e_stop.pub()
        rate.sleep()
            
