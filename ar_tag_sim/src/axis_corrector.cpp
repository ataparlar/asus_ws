#include "ros/ros.h"
#include "tf/tf.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Quaternion.h"
#include <iostream>
#include <cmath>

ros::Publisher pub;

void vis_marker_correct(const visualization_msgs::Marker::ConstPtr& data){
    visualization_msgs :: Marker correct_marker;
    correct_marker = *(data);
    correct_marker.pose.position.x = data->pose.position.z;
    correct_marker.pose.position.y = -data->pose.position.x;
    correct_marker.pose.position.z = data->pose.position.y;
    tf::Quaternion q(
        data->pose.orientation.x,
        data->pose.orientation.y,
        data->pose.orientation.z,
        data->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    tf::Quaternion correct_q;
    correct_q.setRPY(yaw,roll + M_PI_2,-pitch);
    correct_marker.pose.orientation.x = correct_q.getX();
    correct_marker.pose.orientation.y = correct_q.getY();
    correct_marker.pose.orientation.z = correct_q.getZ();
    correct_marker.pose.orientation.w = correct_q.getW();
    pub.publish(correct_marker);
}


int main(int argc, char **argv){
    ros::init(argc,argv,"axis_corrector");
    ros::NodeHandle n;
    ros::Rate rate(100);
    pub = n.advertise<visualization_msgs::Marker>("vis_marker_correct",100);
    ros::Subscriber sub  =n.subscribe("visualization_marker", 100 ,vis_marker_correct);
    ros :: spin();

    return 0;
}