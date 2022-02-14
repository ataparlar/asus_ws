#include <dwb_critics/titrek.h>

#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>
#include <dwb_critics/rotate_to_goal.h>
#include <nav_2d_utils/parameters.h>
#include <nav_core2/exceptions.h>
#include <cmath>
#include <angles/angles.h>


bool double_equals(double a, double b, double epsilon = 0.001)
{
    if(a != 0 && b!= 0){
        return std::abs(a - b) < epsilon;
    } else {
        return false;
    }
}

PLUGINLIB_EXPORT_CLASS(dwb_critics::TitrekCritic, dwb_local_planner::TrajectoryCritic)

namespace dwb_critics
{
    void TitrekCritic::onInit(){
        ROS_WARN_NAMED("Titrek", "Titremiyoz haaa");
        yaw_tolerance_ = nav_2d_utils::searchAndGetParam(critic_nh_, "end_yaw_tolerance", 0.5);
    }

    void TitrekCritic::reset(){
        oscillation_flag = false;
    }

    void TitrekCritic::debrief(const nav_2d_msgs::Twist2D &cmd_vel){
        if (yaws.size() < 8){
            yaws.push_back(cmd_vel.theta);
        } else {
            yaws.erase(yaws.begin());
            yaws.push_back(cmd_vel.theta);
        }

        double a = yaws[3];
        double b = yaws[4];
        
        if((yaws[0] == yaws[1]) && (yaws[1] == yaws[2]) && (yaws[2] == yaws[3]) &&
           (yaws[4] == yaws[5]) && (yaws[5] == yaws[6]) && (yaws[6] == yaws[7]) &&
            double_equals(a,(-1)*b)){
                ROS_WARN_NAMED("Titrek double_equals(a,(-1)*b)", "Titriyoz haa 8");
                oscillation_flag = true;
                oscillation_mode_ = 8;
        } else if((yaws[1] == yaws[2]) && (yaws[2] == yaws[3]) &&
                  (yaws[4] == yaws[5]) && (yaws[5] == yaws[6]) &&
                   double_equals(a,(-1)*b)){
                ROS_WARN_NAMED("Titrek double_equals(a,(-1)*b)", "Titriyoz haa 6");
                oscillation_flag = true;
                oscillation_mode_ = 6;
        }
    }

    bool TitrekCritic::prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel, const geometry_msgs::Pose2D& goal, const nav_2d_msgs::Path2D& global_plan){
        this->global_plan_ = global_plan;
        this->current_pose_ = pose;
        return true;
    }

    double TitrekCritic::scoreTrajectory(const dwb_msgs::Trajectory2D& traj){

        float difference = fabs(angles::shortest_angular_distance(current_pose_.theta, global_plan_.poses.back().theta));

        if(oscillation_flag == true){
            /*if(yaws[7] < 0 && traj.velocity.theta < 0 && traj.velocity.x != 0 && traj.velocity.y != 0){
                return 500.0;
            } else if(yaws[7] > 0 && traj.velocity.theta > 0 && traj.velocity.x != 0 && traj.velocity.y != 0){
                return 500.0;
            }*/

            //This code below works but it turns other way around.
            int index = oscillation_mode_-1;
            if(yaws[index] < 0 && global_plan_.poses[1].theta < 0){
                return 500.0;
            } else if(yaws[index] > 0 && global_plan_.poses[1].theta > 0){
                return 500.0;
            }
            oscillation_flag = false;

            if( difference < yaw_tolerance_){
                if(global_plan_.poses[1].x != 0){
                    return 500.0;
                } 
            }

            //Try to turn the robot towards global plan
            /*start_pose_ = global_plan_.poses[0];
            start_pose_2 = global_plan_.poses[10];
            float start_yaw = atan2(start_pose_2.y-start_pose_.y, start_pose_2.x-start_pose_.x);
            float difference = fabs(angles::shortest_angular_distance(current_pose_.theta, start_yaw));*/
            
        }
        return 0.0;
    }
}//namespace dwb_critics

