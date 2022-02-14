#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


pub = rospy.Publisher('covariance_publisher', PoseWithCovarianceStamped, queue_size=10)
rospy.init_node('imu_covariance_node', anonymous=True)


def CovarianceCallback(data):
    pub.Header, pub.PoseWithCovariance = data.header, data.pose
    pub.publish()


def Imu2Covariance():
    #rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("odometry/filtered", Odometry, CovarianceCallback)
    #while not rospy.is_shutdown():
    #    rate.sleep()
    rospy.spin()



if __name__ == '__main__':
    try:
        Imu2Covariance()
    except rospy.ROSInterruptException:
        pass
