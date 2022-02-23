#!/usr/bin/env python 

#Halil Faruk Karagoz

import numpy as np
import time
import subprocess as subprocess
import rospy
from rospy.rostime import Time
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError 

class hist_eq:
    def __init__(self):
        rospy.init_node("hist_eq");
        self.bridge = CvBridge(); 
        rospy.Subscriber("/camera/image_raw",Image,self.check);
        self.pub = rospy.Publisher("/camera/image_raw1",Image,queue_size=20);

        self.flag_over_exposured = False
        self.flag_low_exposured = False

        self.exposure = 100
        self.gain = 30

        self.upper_limit_gain = False
        self.lower_limit_gain = False
    
        self.upper_limit_exposure = False
        self.lower_limit_exposure = False

        self.use_filter = False

        self.starting_time = rospy.Time.now().secs

        rospy.spin()

    def check(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        now = rospy.Time.now().secs
        
        if (int(now) -int(self.starting_time)) %2 == 0:
            self.camera_settings_opt(cv_image)


       
        
        try:
            ros_img = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        except CvBridgeError as e:
            print(e)
    

        ros_img.header = data.header


        self.pub.publish(ros_img)



    def clahe_YUV(self,cv_image): # improve contrast 
        
        clahe = cv2.createCLAHE(clipLimit = 2.0,tileGridSize = (8,8))
        yuv_img = cv2.cvtColor(cv_image,cv2.COLOR_BGR2YUV)
        y,u,v = cv2.split(yuv_img)
        y_eq = clahe.apply(y)
        yuv_merged = cv2.merge((y_eq,u,v))

        img_final = cv2.cvtColor(yuv_merged,cv2.COLOR_YUV2BGR)

        return img_final

    def clahe_HSV(self,cv_image): # improve contrast 
        
        clahe = cv2.createCLAHE(clipLimit = 2.0,tileGridSize = (8,8))
        hsv_img = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        h,s,v = cv2.split(hsv_img)
        v_eq = clahe.apply(v)
        yuv_merged = cv2.merge((h,s,v_eq))
    
        img_final = cv2.cvtColor(yuv_merged,cv2.COLOR_HSV2BGR)

        return img_final

    
    def gamma_correction(self,image,gamma):
        invgamma = 1.0/gamma
        table = np.array([((i / 255.0) ** invgamma) * 255
            for i in np.arange(0, 256)]).astype("uint8")
    
        return cv2.LUT(image,table)


    
    def biletral_filer(self,img): # edge preserving smoothing filter
        return cv2.bilateralFilter(img,5, 10,10)                                                                                                         



    def camera_settings_opt(self,cv_image):
        hsv_img = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV) 
        h,s,v = cv2.split(hsv_img)
        if np.mean(v)> 160 :
            self.flag_over_exposured = True
        elif np.mean(v) < 60:
            self.flag_low_exposured = True
        else :
            self.flag_over_exposured = False
            self.flag_low_exposured = False

        if (self.flag_over_exposured) and (not ( self.lower_limit_exposure and self.lower_limit_gain)) : # dont run bot gain and exposure is lower limit
            if self.exposure > 10 :
                self.exposure = self.exposure -5
                self.upper_limit_exposure = False
                self.use_filter = False
            else :
                self.lower_limit_exposure = True
                self.use_filter = True
                self.exposure = 4
            if self.gain > 5 :
                self.gain = self.gain = -2
                self.upper_limit_exposure = False
            else : 
                self.lower_limit_gain = True
                self.gain = 5
        
            subprocess.call(['sh', '/home/itu-rover-asus/21rover_ws/src/detect_aruco/script/cam.sh',str(self.exposure),str(self.gain)])


        elif (self.flag_low_exposured) and (not ( self.upper_limit_exposure and self.upper_limit_gain)):
            if self.exposure < 600  :
                self.exposure = self.exposure +5
                self.lower_limit_exposure = False
                self.use_filter = False
            else :
                self.upper_limit_exposure = True
                self.use_filter = False

            if self.gain < 100 :
                self.gain = self.gain + 2
                self.lower_limit_gain = False
            else : 
                self.upper_limit_gain = True
            subprocess.call(['sh', '/home/itu-rover-asus/21rover_ws/src/detect_aruco/script/cam.sh',str(self.exposure),str(self.gain)])
            self.camera_mode = 0


    
            


if __name__ == "__main__":
    hist_eq();

        

