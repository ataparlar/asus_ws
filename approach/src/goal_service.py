#!/usr/bin/env python

from __future__ import print_function
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from approach.srv import SendGoal,SendGoalResponse

def movebase_client(req):
    print("das")
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    goal =req
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return SendGoalResponse(client.get_result())


if __name__ == '__main__':
    rospy.init_node('movebase_client_py')
    result = rospy.Service('send_goal', SendGoal, movebase_client)
    if result:
        rospy.loginfo("Goal execution done!")
    rospy.spin()
   


