#!/usr/bin/env python
import rospy
from rospy.core import is_shutdown, rospyinfo
from std_msgs import msg
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

my_array = Float64MultiArray()
my_array2 = Float64MultiArray()

def joyCallback(my_data):
    my_array.data = my_data.axes
    my_array2.data=my_data.buttons
  
    
def run():
    rospy.init_node('node_name', anonymous=True)
    rate=rospy.Rate(10)
    rospy.Subscriber('joy', Joy, joyCallback)
    pub = rospy.Publisher('multiarray_topic', Float64MultiArray,queue_size=10)
    pub2= rospy.Publisher('multiarray_topic2', Float64MultiArray,queue_size=10)
    
    while not rospy.is_shutdown():
        msg=my_array
        msg2=my_array2
        pub.publish(msg)
        pub2.publish(msg2)
        rate.sleep()
        
    rospy.spin()
    
if __name__ == '__main__': 
    try:
        run()
    except rospy.ROSInterruptException:
        pass
    
    