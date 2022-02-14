#!/usr/bin/env python
import rospy
from rospy.core import is_shutdown, rospyinfo
from std_msgs import msg
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

def callback(data):
    rospy.loginfo("received data")

def listener():
    rospy.init_node("Subscriber_Node", anonymous = True)
    rospy.Subscriber('multiarray_topic', String, callback)
    rospy.spin()
        
if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass