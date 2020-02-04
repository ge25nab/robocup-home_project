#include <grasp/grasp.h>

using namespace GRASP;
void grasp::get_transformation(const geometry_msgs::Point::ConstPtr& msg)
{
    translation_(0) = msg->x;
    translation_(1) = msg->y;
    translation_(2) = msg->z;
}
void grasp::get_rotation(const geometry_msgs::Quaternion::ConstPtr& msg)
{
    q.x() = msg->x;
    q.y() = msg->y;
    q.z() = msg->z;
    q.w() = msg->w;
    rotation_ = q.toRotationMatrix();
}

void grasp::get_desired_pose(const geometry_msgs::Point::ConstPtr& msg)
{

    original(0) = msg->x;
    original(1) = msg->y;
    original(2) = msg->z;
    target = rotation_*original+translation_;

}

bool grasp::grasp_f(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
     
    ros::AsyncSpinner spinner(1);
    spinner.start();


    ROS_INFO_STREAM("You are grasping");
    std::vector<std::string> torso_arm_joint_names;
    //select group of joints
    moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
    //choose your preferred planner
    group_arm_torso.setPlannerId("SBLkConfigDefault");
    group_arm_torso.setPoseReferenceFrame("base_footprint");
    //------these points are for gazebo-------
    // goal_pose.header.frame_id = "base_footprint";
    // goal_pose.pose.position.x = 0.826;
    // goal_pose.pose.position.y = 0.26;
    // goal_pose.pose.position.z = 1.144;
    // goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.60, 0.00, 0.00);
    //------these points are for gazebo-------
    goal_pose.header.frame_id = "base_footprint";
    goal_pose.pose.position.x = target(0)-0.15;
    goal_pose.pose.position.y = target(1);
    goal_pose.pose.position.z = target(2)+0.1;
    goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-1.5, 0.00, 0.00);
    group_arm_torso.setPoseTarget(goal_pose);
    ROS_INFO_STREAM("target position is x:"<<goal_pose.pose.position.x <<"y:"<<goal_pose.pose.position.y <<"z:"<<goal_pose.pose.position.z);
    ROS_INFO_STREAM("Planning to move " <<
                    group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                    group_arm_torso.getPlanningFrame());

    group_arm_torso.setStartStateToCurrentState();
    group_arm_torso.setMaxVelocityScalingFactor(0.3);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //set maximum time to find a plan
    group_arm_torso.setPlanningTime(10.0);
    ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");
    // Execute the plan
    ros::Time start = ros::Time::now();
    ROS_INFO("5");
    group_arm_torso.move();    
    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());    
    spinner.stop();    
    return true;
}

bool grasp::grasp_rectify(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
     
    ros::AsyncSpinner spinner(1);
    spinner.start();


    ROS_INFO_STREAM("You are grasping");
    std::vector<std::string> torso_arm_joint_names;
    //select group of joints
    moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
    //choose your preferred planner
    group_arm_torso.setPlannerId("SBLkConfigDefault");
    group_arm_torso.setPoseReferenceFrame("base_footprint");
    goal_pose.header.frame_id = "base_footprint";
    goal_pose.pose.position.x = target(0)-0.4;
    goal_pose.pose.position.y = target(1);
    goal_pose.pose.position.z = target(2)+0.05;
    goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-1.5, 0.00, 0.00);
    group_arm_torso.setPoseTarget(goal_pose);
    ROS_INFO_STREAM("target position is x:"<<goal_pose.pose.position.x <<"y:"<<goal_pose.pose.position.y <<"z:"<<goal_pose.pose.position.z);
    ROS_INFO_STREAM("Planning to move " <<
                    group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                    group_arm_torso.getPlanningFrame());

    group_arm_torso.setStartStateToCurrentState();
    ROS_INFO("1");
    group_arm_torso.setMaxVelocityScalingFactor(0.3);
    ROS_INFO("2");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    ROS_INFO("3");
    //set maximum time to find a plan
    group_arm_torso.setPlanningTime(5.0);
    ROS_INFO("4");
    // bool success = (bool) group_arm_torso.plan(my_plan);
    // ROS_INFO("5");
    // if ( !success )
    //     throw std::runtime_error("No plan found");
    ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");
    // Execute the plan
    ros::Time start = ros::Time::now();
    ROS_INFO("5");
    group_arm_torso.move();
    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
    
    spinner.stop();
    
    return true;
}