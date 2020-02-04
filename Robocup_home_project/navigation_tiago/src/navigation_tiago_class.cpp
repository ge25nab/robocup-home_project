#include <navigation_tiago/navigation_tiago.h>

using namespace NAVIGATIONTIAGO;

int Navigation_tiago::movearm(std::map<std::string, double> target_position)
{

	ros::NodeHandle n;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	std::vector<std::string> torso_arm_joint_names;
	//select group of joints
	moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
	//choose your preferred planner
	group_arm_torso.setPlannerId("SBLkConfigDefault");

	torso_arm_joint_names = group_arm_torso.getJoints();

	group_arm_torso.setStartStateToCurrentState();
	group_arm_torso.setMaxVelocityScalingFactor(0.3);

	for (unsigned int i = 0; i < torso_arm_joint_names.size(); ++i)
		if ( target_position.count(torso_arm_joint_names[i]) > 0 )
		{
		ROS_INFO_STREAM("\t" << torso_arm_joint_names[i] << " goal position: " << target_position[torso_arm_joint_names[i]]);
		group_arm_torso.setJointValueTarget(torso_arm_joint_names[i], target_position[torso_arm_joint_names[i]]);
		}

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	group_arm_torso.setPlanningTime(5.0);
	bool success = (bool)group_arm_torso.plan(my_plan);

	if ( !success )
		throw std::runtime_error("No plan found");

	ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

	// Execute the plan
	ros::Time start = ros::Time::now();

	group_arm_torso.move();

	ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

	spinner.stop();

	return EXIT_SUCCESS;
}

move_base_msgs::MoveBaseGoal Navigation_tiago::Next_point(int id) 
{	
	//define the navigation
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = Targetx[id];
	goal.target_pose.pose.position.y = Targety[id];
	goal.target_pose.pose.position.z = 0.0;
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = Targetz[id];
	goal.target_pose.pose.orientation.w = Targetw[id];

	return goal;

}		