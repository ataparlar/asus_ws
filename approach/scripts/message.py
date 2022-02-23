#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int64, Header 
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox

class TurningWay:
    def __init__(self):
        rospy.init_node("way_of_turning_node")

        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes , self.calculate)
        pub = rospy.Publisher('/turningway', String, queue_size=10)

        rate = rospy.Rate(1)
        self.threshold = 640/2 
        self.way = String()
        self.way.data = "" 
        
        while not rospy.is_shutdown():
            pub.publish(self.way)
            rate.sleep()
        

    def calculate(self, data):

        for box in data.bounding_boxes:
            rospy.loginfo(
                "Xmin: {}, Xmax: {} Ymin: {}, Ymax: {}".format(
                    box.xmin, box.xmax, box.ymin, box.ymax
                )   
            )
            self.avgx = (box.xmax + box.xmin)/2
            if self.avgx < self.threshold - 50:
                self.way.data = "left"
            elif self.avgx > self.threshold + 50:
                self.way.data = "right"
            else:
                self.way.data = "okay" 


if __name__ == "__main__":
    try:
        TurningWay()
        
    except KeyboardInterrupt:
        rospy.signal_shutdown()
