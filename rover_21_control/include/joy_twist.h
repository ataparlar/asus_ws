/**
 * Author: Busra Asan
 * Date: 24/12/2021
*/

#ifndef JOYTWIST_H
#define JOYTWIST_H

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include <iostream>

/**
 * @brief: Configurable joystick to twist node
 * @param enable_turbo: Set 1 to use turbo mode
 * @param turbo_button: Choose turbo button
 * @param turbo_coefficient: velocity = turbo_coefficient*axis_value
 * @param movement_button: Choose main movement button range
 * @param linear_axis: Choose linear axis
 * @param angular_axis: Choose angular axis 
*/

class joyNode{
private:
    ros::Subscriber joystickSub;
    geometry_msgs::Twist twist;
    int enable_turbo;
    double turbo_coefficient;
    int turbo_button;
    int movement_button;
    int linear_axis;
    int angular_axis;
    
public:
    joyNode(ros::NodeHandle& nh);
    ros::Publisher twistPub;
    void joyCallback(const sensor_msgs::Joy& data);
};

#endif