#!/usr/bin/env python

from math import sin, cos, pi
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class OdometryHandler():
    def __init__(self):
        rospy.init_node("odom_calculator",anonymous=False)

        self.odom_pub = rospy.Publisher('/odometry/wheel', Odometry, queue_size = 10)
        rospy.Subscriber('/rover_serial_encoder', String, self.serial_callback)
        rospy.Subscriber('/imu1/data', Imu, self.imu_cb)

        self.CIRCUMFERENCE = 0.14 * pi * 2
        self.FREQUENCY = rospy.Rate(10)

        self.x = 0.0
        self.y = 0.0
        
        self.yaw_change = 0.0
        self.init_yaw = 0.0
        self.yawCounter = 0
        self.last_yaw = 0.0
        self.curr_yaw = 0.0
        self.angular_velocity = None
        self.imuFlag = False

        self.current_time = None
        self.last_time = rospy.Time.now()
        self.dt = None

        self.vx = 0.0
        self.right_wheels = 0.0
        self.left_wheels = 0.0
        self.wheelVels = [0,0,0,0] # Front Left, Back Left, Front Right, Back Right
        self.encoder_data = ""

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_link"

        self.main()


    def imu_cb(self,data):
        self.last_yaw = self.curr_yaw
        [self.curr_roll, self.curr_pitch, self.curr_yaw] = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]) 
        
        if(self.imuFlag):
            if (self.yawCounter < 5):
                self.init_yaw += self.curr_yaw
                self.yawCounter += 1
            elif(self.yawCounter == 5):
                self.imuFlag = True
                self.init_yaw /= 5
        
        else:
            self.angular_velocity = data.angular_velocity
            self.yaw_change = self.curr_yaw - self.init_yaw

    def serial_callback(self,data):
        self.encoder_data = data.data
        self.splitted = data.data.split(',')

        if(self.splitted[0] == 'A' and self.splitted[-1] == 'B'):
            for i in range (1,4):
                if (float(self.splitted[i]) >= 1000):
                    self.wheelVels[i] = self.CIRCUMFERENCE * (-(float(self.splitted[i])-1000)) / 60
                else:
                    self.wheelVels[i] = self.CIRCUMFERENCE * (float(self.splitted[i])) / 60
    
    def publishOdom(self):
        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.q = quaternion_from_euler(0, 0, self.yaw_change)
        self.odom_msg.pose.pose.orientation.x = self.q[0]
        self.odom_msg.pose.pose.orientation.y = self.q[1]
        self.odom_msg.pose.pose.orientation.z = self.q[2]
        self.odom_msg.pose.pose.orientation.w = self.q[3]
        self.odom_msg.twist.twist.linear.x = self.vx
        self.odom_pub.publish(self.odom_msg)

    def main(self):

        while not rospy.is_shutdown():
            self.current_time = rospy.Time.now()
            self.dt = (self.current_time - self.last_time).to_sec()
            self.last_time = self.current_time

            self.left_wheels = self.wheelVels[0] + self.wheelVels[1] / 2
            self.right_wheels = self.wheelVels[2] + self.wheelVels[3] / 2

            self.vx = self.left_wheels + self.right_wheels / 2

            self.delta_x = self.vx * cos(self.yaw_change) * self.dt
            self.delta_y = self.vx * sin(self.yaw_change) * self.dt

            self.x += self.delta_x
            self.y += self.delta_y

            rospy.loginfo("x: %s y: %s",self.x,self.y)

            self.publishOdom()
            self.FREQUENCY.sleep()

if __name__ == "__main__":
    try:
        OdometryHandler()
    except KeyboardInterrupt:
        rospy.signal_shutdown()


