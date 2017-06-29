#include "moveit_interface.ih"

std::vector<double> MoveItInterface::calculateIK(geometry_msgs::Pose pose)
{
	// Create robot state
	moveit_msgs::RobotState rs;
	rs.joint_state = d_joint_state;

	// Create pose stamped object
	geometry_msgs::PoseStamped stamped_pose;
	stamped_pose.header.frame_id = "mico_base_link";
	stamped_pose.pose = pose;

	// Create moveit IK service object
	moveit_msgs::GetPositionIK srv;
	srv.request.ik_request.group_name = d_group_name;
	srv.request.ik_request.avoid_collisions = true;
	srv.request.ik_request.robot_state = rs;
	srv.request.ik_request.timeout = ros::Duration(45);
	srv.request.ik_request.ik_link_name = getEndEffectorLink();
	srv.request.ik_request.pose_stamped = stamped_pose;

	/*std::vector<double> tolerance_pose(3, 0.03);
	std::vector<double> tolerance_angle(3, 0.03);
	moveit_msgs::Constraints constraint = kinematic_constraints::constructGoalConstraints(getEndEffectorLink(), pose, tolerance_pose, tolerance_angle);
	moveit_msgs::JointConstraint mico_shoulder_yaw_joint_constraint;
	mico_shoulder_yaw_joint_constraint.joint_name = "mico_shoulder_yaw_joint";
	mico_shoulder_yaw_joint_constraint.position = -1.57;
	mico_shoulder_yaw_joint_constraint.tolerance_above = 1.25;
	mico_shoulder_yaw_joint_constraint.tolerance_below = 1.25;
	mico_shoulder_yaw_joint_constraint.weight = 1;
	constraint.joint_constraints.push_back(mico_shoulder_yaw_joint_constraint);
	srv.request.ik_request.constraints = constraint;*/

	ROS_INFO("Computing IK for given pose");
	if (d_ik_compute_service.call(srv))
	{
		// Check if it was successful
		if (srv.response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
		{
			ROS_INFO("Calculating IK successful");
			return srv.response.solution.joint_state.position;
		}

		// Unsuccessful
		ROS_WARN("Could not find IK solution");
		return std::vector<double>();
	}

	ROS_INFO("Service call for computing IK failed");
	return std::vector<double>();
}
