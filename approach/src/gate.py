#!/usr/bin/env python
import rospy
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Transform
from nav_msgs.msg import Odometry 
import math as m
import numpy as np
from tf.transformations import euler_from_quaternion
from send_goal import send_goal
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Bool


class Gate:
    def __init__(self) :
        rospy.init_node("gate")
        self.x,self.y,self.z = None,None,None
        rospy.Subscriber("/odometry/filtered",Odometry,self.odom)
        rospy.Subscriber("/fiducial_transforms",FiducialTransformArray,self.marker)
        rospy.Subscriber("move_base/status",GoalStatusArray,self.move_base_status)
        self.pub_e_stop = rospy.Publisher("/e_stop",Bool,queue_size=10)

        self.goal_number = 2
        self.initalized_look = False # need to find points just once if points are found then it will be true
        self.initalized_finish = False
        self.flag_draw_circle = False
        self.done = False
        self.yaw = 0
        self.circle_pts = []
        self.markers = {}
        self.count_marker = {}
        self.main_pts = [] # look points
        self.trial = 0
        self.starting_time = rospy.Time.now().to_sec()
        self.Rate = rospy.Rate(1)
        self.run()


    def odom(self,data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z
        self.x_or = data.pose.pose.orientation.x
        self.y_or = data.pose.pose.orientation.y
        self.z_or = data.pose.pose.orientation.z
        self.w_or = data.pose.pose.orientation.w
        self.roll,self.pitch,self.yaw = euler_from_quaternion((self.x_or,self.y_or,self.z_or,self.w_or))

    def run(self):
        while not rospy.is_shutdown():
            if self.done == False:
                if rospy.Time.now().to_sec() - self.starting_time >2 :
                    if (self.initalized_look == False and len(self.markers) == 1):
                        print("saw only one tag")
                        self.marker_x,self.marker_y = list(self.markers.values())[0].translation.x,list(self.markers.values())[0].translation.y
                        self.circle_points((self.marker_x,self.marker_y),5,m.pi/3)
                        self.initalized_look = True
                        self.flag_draw_circle = True
                    if(self.initalized_look and len(self.markers) == 1 ):
                        self.draw_circle()
                        print("looking for second")
                    if(len(self.markers) == 2 and not self.initalized_finish):
                        print("found both")
                        rospy.sleep(2)
                        self.gate_points(4)
                        self.initalized_finish = True
                        print("start_finish")
                    if(self.initalized_finish and len(self.markers) == 2):
                        self.finish_job()
                    self.Rate.sleep()


    def gate_points(self,r):
        middle_x =  (list(self.markers.values())[0].translation.x + list(self.markers.values())[1].translation.x) / 2
        middle_y =  (list(self.markers.values())[0].translation.y + list(self.markers.values())[1].translation.y) / 2
        self.middle_p = (middle_x,middle_y)
        angle = m.atan2((list(self.markers.values())[0].translation.y - list(self.markers.values())[1].translation.y),(list(self.markers.values())[0].translation.x - list(self.markers.values())[1].translation.x))
        self.edge_pts = ((m.cos(angle) * (r+1)  + middle_x ,m.sin(angle) * (r+1) + middle_y))
        angle += m.pi/2
        self.main_pts.append((m.cos(angle) * r + middle_x ,m.sin(angle) * r + middle_y))
        self.main_pts.append((-m.cos(angle) * r + middle_x ,-m.sin(angle) * r + middle_y))
        dist1 = self.find_distance((self.x,self.y),self.main_pts[0])
        dist2 = self.find_distance((self.x,self.y),self.main_pts[1])
        print(self.edge_pts)
        if(dist1 < dist2):
            self.first_x,self.first_y = self.main_pts[0][0],self.main_pts[0][1]
            self.next_x,self.next_y = self.main_pts[1][0],self.main_pts[1][1]
        else:
            self.first_x,self.first_y = self.main_pts[1][0],self.main_pts[1][1]
            self.next_x,self.next_y = self.main_pts[0][0],self.main_pts[0][1]

        print((list(self.markers.values())[0].translation.x) , (list(self.markers.values())[0].translation.y))
        print((list(self.markers.values())[1].translation.x) , (list(self.markers.values())[1].translation.y))
        print(middle_x,middle_y)
       
        if self.find_distance((self.x,self.y),(self.first_x,self.first_y)) > 1:
            send_goal(self.first_x,self.first_y,m.atan2(middle_y-self.first_y,middle_x-self.first_x))
        else:
            send_goal(self.x,self.y,m.atan2(middle_y-self.y,middle_x-self.x))
            


    def find_distance(self,pt1,pt2):
        return np.sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)

    def finish_job(self):
        if self.status == 3 and self.trial == 0:
            rospy.sleep(3)
            print(self.is_it_right_side(0.5))
            if self.is_it_right_side(0.5):
                print("found right side passing the gate!")
                send_goal(self.middle_p[0],self.middle_p[1],self.yaw)
                send_goal(self.next_x,self.next_y,self.yaw)
                self.done = True
            else:
                print("Wrong side !")
                send_goal(self.edge_pts[0],self.edge_pts[1],m.atan2(self.next_y-self.edge_pts[1],self.next_x-self.edge_pts[0]))
            self.trial+=1
        if self.status == 3 and self.trial == 1:
            print("Go check other side !")
            send_goal(self.next_x,self.next_y,m.atan2(self.middle_p[1]-self.next_y,self.middle_p[0]-self.next_x))
            self.trial+=1
        if self.status == 3 and self.trial ==2:
            rospy.sleep(3)
            if self.is_it_right_side(0.5):
                print("found right side passing the gate!")
                send_goal(self.middle_p[0],self.middle_p[1],self.yaw)
                send_goal(self.first_x,self.first_y,self.yaw)
                self.trial+=1
            else:
                self.markers = {}
                self.trial = 0
                self.initalized_finish = False
        if(self.status == 3) and (self.trial== 3):
            send_goal(self.first_x,self.first_y,self.yaw)

    
    def is_it_right_side(self,threshold):
        marker1_ori = list(self.markers.values())[0].rotation
        marker2_ori = list(self.markers.values())[1].rotation
        marker1_yaw = euler_from_quaternion((marker1_ori.x,marker1_ori.y,marker1_ori.z,marker1_ori.w))[1]
        marker2_yaw = euler_from_quaternion((marker2_ori.x,marker2_ori.y,marker2_ori.z,marker2_ori.w))[1]
        print(marker1_yaw,marker2_yaw)
        print((m.pi - (marker1_yaw % m.pi)))
        #if abs(marker1_yaw - marker2_yaw) < threshold :
        if (marker1_yaw % m.pi < threshold) or ((m.pi - (marker1_yaw % m.pi)) < threshold):
            if (marker2_yaw % m.pi < threshold) or ((m.pi - (marker2_yaw % m.pi)) < threshold):
                return True  
        return False


    def stop(self):
        rospy.loginfo("I stopped")
        for i in range(10):
            self.pub_e_stop.publish(True)
        
    def go_on(self):
        rospy.loginfo("I go on")
        for i in range(10):
            self.pub_e_stop.publish(False)
    


    def find_first_p(self,p1,p2,r):
        x1,y1 = p1 # vehicle
        x2,y2 = p2 # marker
        angle = m.atan2(y2-y1,x2-x1)
        relative_y = m.sin(angle) * r
        relative_x = m.cos(angle) * r
        send_goal(self.x,self.y,angle -m.pi/2)
        return(relative_x ,relative_y)
        

    
    def marker(self,data):
        for i in data.transforms:
            self.correct_transform = Transform()
            transform = i.transform
            self.correct_transform.translation.x = transform.translation.z
            self.correct_transform.translation.y = - transform.translation.x
            self.correct_transform.rotation = transform.rotation
            if i.fiducial_id not in self.count_marker.keys():
                self.count_marker[i.fiducial_id] =0
                send_goal(0,0,m.atan2(self.correct_transform.translation.y,self.correct_transform.translation.x),"base_link")
                self.stop()
                print("I might see a tag just checking!")
                rospy.sleep(4)
                self.go_on()
            for key,value in self.count_marker.items():
                if key == i.fiducial_id:
                    self.count_marker[i.fiducial_id] +=1
                if value > 15:
                    self.correct_transform.translation.x += 0.5 # since camera is 50 cm in front of base link
                    #print("marker : " + str(pose.translation.x)  + "  " + str(pose.translation.y))    
                    dist_from_vehicle = self.find_distance((0,0),(self.correct_transform.translation.x,self.correct_transform.translation.y))  
                    #print(dist_from_vehicle)
                    beta = m.pi -  m.atan2(self.correct_transform.translation.x,self.correct_transform.translation.y)
                    #print(beta)
                    #print(self.yaw)
                    alpha = self.yaw -  (m.pi/2  - beta)
                    #print(alpha)
                    x_rel = (dist_from_vehicle ) * m.cos(alpha)
                    y_rel = (dist_from_vehicle ) * m.sin(alpha)
                    #print("marker : " + str( x_rel)  + "  " + str(y_rel))
                    if(self.x != None):
                        self.correct_transform.translation.x = x_rel + self.x
                        self.correct_transform.translation.y = self.y +  y_rel
                        #print("marker last : " + str( pose.translation.x)  + "  " + str(pose.translation.y))
                        self.markers[i.fiducial_id] = self.correct_transform
                    break
        #print("I am sure there are " + str(len(self.markers)) + " markers" )
    
    def circle_points(self,center_p,r,angle):
        self.circle_angle = angle
        rotation_matrix = np.array(([m.cos(angle),-m.sin(angle)],[m.sin(angle),m.cos(angle)]))
        first_p = self.find_first_p((self.x,self.y),center_p,r)
        last_p = np.array((first_p))
        for i in range(int(2 * m.pi / angle) + 1):
            last_p = np.matmul(rotation_matrix,last_p)
            self.circle_pts.append((last_p[0] + center_p[0],last_p[1] + center_p[1]))
        

    def move_base_status(self,data):
        self.stat_list = data.status_list
        for i in self.stat_list :
            self.status = i.status
            self.last_goal_id = i.goal_id
        
    
    def draw_circle(self):
        if (self.status == 3 and  (self.flag_draw_circle)) :
            rospy.loginfo("Get a new Goal")
            self.goal_number += 1
            x,y = self.circle_pts[self.goal_number]
            angle = m.atan2(self.marker_y - y,self.marker_x-x) 
            send_goal(x,y,angle)


if __name__ == "__main__":
    Gate()