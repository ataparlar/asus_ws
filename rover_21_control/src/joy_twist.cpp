/**
 * Author: Busra Asan
 * Date: 24/12/2021
*/

#include "../include/joy_twist.h"

joyNode::joyNode(ros::NodeHandle& nh){
    joystickSub = nh.subscribe("/joy", 10, &joyNode::joyCallback, this);
    twistPub = nh.advertise<geometry_msgs::Twist>("/joy_teleop/cmd_vel", 10);
    nh.param("/joy_twist/enable_turbo", enable_turbo, 1);
    nh.param("/joy_twist/turbo_button", turbo_button, 6);
    nh.param("/joy_twist/turbo_coefficient", turbo_coefficient, 1.5);
    nh.param("/joy_twist/movement_button", movement_button, 4);
    nh.param("/joy_twist/linear_axis", linear_axis, 1);
    nh.param("/joy_twist/angular_axis", angular_axis, 0);

    ros::spin();
}

void joyNode::joyCallback(const sensor_msgs::Joy& data){
    ROS_INFO("sss");
    if(linear_axis != 1 && linear_axis != 3)
    {
        ROS_ERROR("Linear axis value should be 1 for left stick or 3 for right stick.");
    } 
    else if (angular_axis != 0 && angular_axis != 2)
    {
        ROS_ERROR("Angular axis value should be 0 for left stick or 2 for right stick.");
    }
    else if( (movement_button < 0 || movement_button > 11) || (turbo_button < 0 || turbo_button > 11) )
    {
        ROS_ERROR("Wrong button configuration. Button value range: (0,11)");
    }
    else if(movement_button == turbo_button)
    {
        ROS_ERROR("Wrong button configuration. Turbo and Movement buttons should not be equal.");
    }
    else
    {
        if(data.buttons[turbo_button]==1 && enable_turbo){ //turbo mode
            twist.linear.x = data.axes[1]*turbo_coefficient;
            twist.angular.z = data.axes[0]*turbo_coefficient;
        }
        else if(data.buttons[movement_button] == 1){
            twist.linear.x = data.axes[linear_axis];
            twist.angular.z = data.axes[angular_axis];
        } else {
            twist.linear.x = 0;
            twist.angular.z = 0;
        }
        twistPub.publish(twist);
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "joystick_twist_node");
    ros::NodeHandle nh;
    joyNode joy(nh);
    
    return 0;
}