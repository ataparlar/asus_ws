#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped


pub = rospy.Publisher('covariance_publisher', PoseWithCovariance, queue_size=10)
rospy.init_node('imu_covariance_node', anonymous=True)


def CovarianceCallback(data):
    pub.publish(data.pose)


def Imu2Covariance():
    #rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, CovarianceCallback)
    #while not rospy.is_shutdown():
    #    rate.sleep()
    rospy.spin()



if __name__ == '__main__':
    try:
        Imu2Covariance()
    except rospy.ROSInterruptException:
        pass
