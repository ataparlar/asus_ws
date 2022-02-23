import rospy
from move_base_msgs.msg import MoveBaseGoal,MoveBaseAction
import actionlib
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

class send_goal:
    def __init__(self,x, y, yaw, frame_id='odom',wait_flag = True):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.frame_id = frame_id
        self.wait_flag = wait_flag  
        pose = self.create_geo_pose_stamped(self.x,self.y,self.yaw,self.frame_id)
        self.goal = MoveBaseGoal(target_pose = pose)
        self.movebase_client(self.goal)


    def create_geo_pose_stamped(self,x, y, yaw, frame_id):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w = quaternion_from_euler(0,0,yaw)
        pose.header.frame_id = frame_id
        self.last_stamp =  rospy.Time.now()
        pose.header.stamp =self.last_stamp
        return pose

    def movebase_client(self,goal):
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()
        self.got_plan = rospy.Time.now().to_sec()
        client.send_goal(goal)
        if self.wait_flag :
            wait = client.wait_for_result()
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
                return client.get_result()