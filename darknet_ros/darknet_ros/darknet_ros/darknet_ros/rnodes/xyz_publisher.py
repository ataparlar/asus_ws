#!/usr/bin/env python2
# coding:utf-8

#Depth camera must be started using: roslaunch realsense2_camera rs_camera.launch align_depth:=true, rosbag böyle alınmamış

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
#import ros_numpy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg        import PointCloud2
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes, Coordinate, ObjectCount
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CompressedImage
from PIL import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo as cameraInfo
import math
import tf
import pyrealsense2 as rs
from darknet_ros_msgs.srv import srvCoordinate as Coordinate_srv
from scipy.ndimage import gaussian_filter
import pyrealsense2
import math

class xyz_publisher:
    def __init__(self):

        rospy.init_node('depth_image_listener', anonymous=False)

        self.bounding_box = np.array((0,0,0,0))
        #variables for camera parameters
        self.camera_info = np.array((0,0,0,0))
        self.ppx = 0.0
        self.ppy = 0.0
        self.fx = 0.0
        self.fy = 0.0
        #found is used to check whether probe exists
        self.found = False
        self.coordinates = Coordinate()
        self.arr2 = np.zeros((640,480))
        #used for finding the two diagonals of the bounding box
        self.positive_diagonal = np.array((0,0,0))
        self.negative_diagonal = np.array((0,0,0))
        self.point = np.array((0.0,0.0,0.0))

        self.center_x_pixel = 0.0
        self.center_y_pixel = 0.0
        self.xmin = 0
        self.xmax = 0
        self.ymin = 0
        self.ymax = 0

        self.probe_y = 0
        self.donder = 0

        self.pixel_x = 0
        self.pixel_y = 0
        self.depth = 0
        self.camera_object = 0

        self.i = 0
        self.gaus_array = np.zeros((1,64))
        self.gaus_array_y = np.zeros((1,64))

        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", msg_Image, self.point_callback)
        self.camera_sub = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", cameraInfo, self.camera_callback)
        self.bounding_box_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.bounding_boxes_callback)
        self.object_count_sub = rospy.Subscriber("darknet_ros/found_object", ObjectCount, self.object_callback)
        self.pub = rospy.Publisher('/probe/xyz_coordinates', Coordinate, queue_size=1)
        self.donder_pub = rospy.Publisher('/probe/frame', String, queue_size=1)
        self.detection_pub = rospy.Publisher('/probe/detection', String, queue_size=1)
        self.servy = rospy.Service('basan_xyz_coordinates', Coordinate_srv, self.coordinates_callback)
            
        self.rate = rospy.Rate(10) #10Hz

        #count = 0
        #find_count = 0
        while not rospy.is_shutdown():
            #count+=1
            # gaus_array = np.zeros((1,64))
            # gaus_array_y = np.zeros((1,64))
            # distance_avg = 0
            # y_avg = 0
            print("coordinates: ", self.coordinates)
            if(self.found == True):
                self.detection_pub.publish("probe detected")
                self.gaus_array[0][self.i] = self.coordinates.x
                self.gaus_array_y[0][self.i] = self.coordinates.y
                self.i += 1
                if self.i == 63:
                    self.loopy(self.gaus_array, self.gaus_array_y)
                    self.tf_broadcaster(self.coordinates)
                    self.pub.publish(self.coordinates)
                    self.donder = self.seek_probe(self.probe_y, 290, 350) 
                    self.donder_pub.publish(str(self.donder))
                    self.i = 0
                # #find_count += 1
                #print "probe detected"
                # self.detection_pub.publish("probe detected")
                # for i in range(0,64):
                #     gaus_array[0][i] = self.coordinates.x
                #     gaus_array_y[0][i] = self.coordinates.y
                #     if(i == 63):
                #         gaus_array = gaus_array.reshape((8,8))
                #         filtered_array = gaussian_filter(gaus_array, sigma=0.7)
                #         gaus_array = gaus_array.reshape((1,64))

                #         gaus_array_y = gaus_array_y.reshape((8,8))
                #         filtered_array_y = gaussian_filter(gaus_array_y, sigma=0.7)
                #         gaus_array_y = gaus_array_y.reshape((1,64))
                #         print(gaus_array_y, "after")

                #         for j in range(0,64):
                #             # if j != 0:
                #             #     #print(j, gaus_array_y[0][j])
                #             distance_avg += gaus_array[0][j]
                #             y_avg += gaus_array_y[0][j]
                #             if(j == 63):
                #                 distance_avg /= 64
                #                 y_avg /= 64
                #                 self.coordinates.x = distance_avg
                #                 self.coordinates.y = y_avg
                #                 self.tf_broadcaster(self.coordinates)
                #                 self.pub.publish(self.coordinates)
                #                 self.donder = self.seek_probe(self.probe_y, 290, 350) 
                #                 self.donder_pub.publish(str(self.donder))
            else:
                print "no probe"
                self.detection_pub.publish("No Probe")
            
            #print (float(find_count) / float(count))*100
            #print count
            self.rate.sleep()  

    def loopy(self, gaus_array, gaus_array_y):
        distance_avg = 0
        y_avg = 0
        gaus_array = gaus_array.reshape((8,8))
        filtered_array = gaussian_filter(gaus_array, sigma=1)
        gaus_array = filtered_array.reshape((1,64))

        gaus_array_y = gaus_array_y.reshape((8,8))
        filtered_array_y = gaussian_filter(gaus_array_y, sigma=1)
        gaus_array_y = filtered_array_y.reshape((1,64))

        for j in range(0,64):
            # if j != 0:
            #     #print(j, gaus_array_y[0][j])
            distance_avg += gaus_array[0][j]
            y_avg += gaus_array_y[0][j]
            if(j == 63):
                distance_avg /= 64
                y_avg /= 64
                self.coordinates.x = distance_avg
                self.coordinates.y = y_avg

    def seek_probe(self, probe_center_y, frame_min, frame_max):
        #print(probe_center_y)
        if frame_min < probe_center_y and probe_center_y < frame_max:
            return 0
        else:
            if probe_center_y - 320 > 0:
                return 1
            else:
                return -1

    def coordinates_callback(self, data):
        if(data.gimme == True):
            return self.coordinates

    def find_center(self):

        center_x_pixel = self.bounding_box[2] - (self.bounding_box[2] - self.bounding_box[0])/2
        center_y_pixel = self.bounding_box[3] - (self.bounding_box[3] - self.bounding_box[1])/2


    def calculate_diagonal(self, x1,y1,x2,y2):

        #find the slope of a linear function
        if( (x1-x2) != 0):
            m = (y1-y2)/(x1-x2)
        else:
            m = 0
        b = y1 - m*x1
        return m, b

    def calculate_diagonal_average(self, a,b,c,d): 
        #sum all the points on the diagonal and take the average depth

        m1, b1 = self.calculate_diagonal(a,b,c,d)
        counter = 0
        pointt =  np.array((0.0,0.0,0.0))
        for i in range(self.xmin, self.xmax):
            y_value = i*m1 + b1
            if(y_value == 480):
                y_value -= 1
            pointt[0] += i
            pointt[1] += y_value
            pointt[2] += self.arr2[y_value][i]
            counter+=1

        if(counter != 0):
            pointt = np.divide(pointt, counter)

        return pointt


    def bounding_boxes_callback(self, data):

        self.xmin = data.bounding_boxes[0].xmin 
        self.ymin = data.bounding_boxes[0].ymin
        self.xmax = data.bounding_boxes[0].xmax 
        self.ymax = data.bounding_boxes[0].ymax

    def convert_depth_to_phys_coord_using_realsense(self):
        x = self.pixel_x
        y = self.pixel_y
        z = self.depth
        cameraInfo = self.camera_object
        _intrinsics = pyrealsense2.intrinsics()
        _intrinsics.width = self.camera_object.width
        _intrinsics.height = self.camera_object.height
        _intrinsics.ppx = 354.54743
        _intrinsics.ppy = 273.56777
        _intrinsics.fx = 631.11981
        _intrinsics.fy = 631.39136

        # _intrinsics.ppx = self.camera_object.K[2]
        # _intrinsics.ppy = self.camera_object.K[5]
        # _intrinsics.fx = self.camera_object.K[0]
        # _intrinsics.fy = self.camera_object.K[4]
        #_intrinsics.model = cameraInfo.distortion_model
        _intrinsics.model  = pyrealsense2.distortion.none
        _intrinsics.coeffs = [i for i in cameraInfo.D]
        result = pyrealsense2.rs2_deproject_pixel_to_point(_intrinsics, [x, y], self.depth)
        #result[0]: right, result[1]: down, result[2]: forward
        return result[2], -result[0], -result[1]


    def point_callback(self, data):

        #function to take pixel coordinates and depth of the probe and convert it to a real world xyz point

        bridge = CvBridge()
        depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        self.arr2 = np.array(depth_image, dtype=np.float32)
        #self.arr2 = data

        self.negative_diagonal = self.calculate_diagonal_average(self.xmin, self.ymin, self.xmax, self.ymax)
        self.positive_diagonal = self.calculate_diagonal_average(self.xmin, self.ymax, self.xmax, self.ymin)

        #Take the diagonal which has the smaller average depth value, the closer one.
        if(self.negative_diagonal[2] < self.positive_diagonal[2]):
            self.point = self.negative_diagonal
        else:
            self.point = self.positive_diagonal

        self.pixel_x = self.point[0]
        self.pixel_y = self.point[1]
        self.depth = self.point[2]

        coordinatesto = self.convert_depth_to_phys_coord_using_realsense()
        #print(coordinatesto)

        self.probe_y = self.point[0]
        #print(self.point[0], self.point[1])
        #Use camera intrinsincs to convert pixel coordinates to real world xyz
        self.point[0] = (self.point[0] - self.camera_info[0])*self.point[2] / self.camera_info[2]
        self.point[1] = (self.point[1] - self.camera_info[1])*self.point[2] / self.camera_info[3]
        self.point[2] = self.point[2]

        # self.coordinates.x = self.point[2] #distance
        # self.coordinates.y = self.point[0] #horizontal
        # self.coordinates.z = self.point[1]*(-1) #vertical

        depth = coordinatesto[0]
        z = math.sqrt(pow(depth,2) - pow(coordinatesto[1],2) - pow(coordinatesto[2],2))
        #print(depth, z)
        self.coordinates.x = z/9.0 #distance
        #print(self.coordinates.x)
        self.coordinates.y = coordinatesto[1]/10.0 #horizontal
        self.coordinates.z = coordinatesto[2]/10.0 #vertical


    def camera_callback(self, cameraData):

        #Set intrinsinc values of the camera

        self.camera_object = cameraData
        # self.camera_info[0] = cameraData.K[2] #ppx
        # self.camera_info[1] = cameraData.K[5] #ppy
        # self.camera_info[2] = cameraData.K[0] #fx
        # self.camera_info[3] = cameraData.K[4] #fy

        #self.camera_info[0] = 354.54743 #ppx
        self.camera_info[0] = 354.54743
        self.camera_info[1] = 273.56777 #ppy
        self.camera_info[2] = 631.11981 #fx
        self.camera_info[3] = 631.39136 #fy
        
        # _intrinsics.height = self.camera_object.height
        # _intrinsics.ppx = 354.54743
        # _intrinsics.ppy = 273.56777
        # _intrinsics.fx = 631.11981
        # _intrinsics.fy = 631.39136

    def object_callback(self, data):

        #Set whether probe exists

        if(data.count > 0):
            self.found = True
        else:
            self.found = False

    def tf_broadcaster(self, coordinates):
        br = tf.TransformBroadcaster()
        br.sendTransform(( coordinates.x/1000, coordinates.y/1000, coordinates.z/1000), (0.0,0.0,0.0,1.0), rospy.Time(), 'probe', '/camera_link')


if __name__ == '__main__':

    try:
        xyz_publisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")
    
