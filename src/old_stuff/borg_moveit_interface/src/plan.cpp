#include "moveit_interface.ih"

void MoveItInterface::planPoseGoal(geometry_msgs::Pose goal)
{
	ROS_INFO("Planning XYZ-quaternion goal!");

	ROS_INFO_STREAM("EEF: "<< d_arm.getEndEffectorLink());

	// Set target pose
    d_arm.stop();
    d_arm.clearPoseTargets();
//	d_arm.setPositionTarget(goal.position.x, goal.position.y, goal.position.z);
//	d_arm.setOrientationTarget(goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w);
    


	ROS_INFO_STREAM("POSE pos: " << goal.position.x << ", " << goal.position.y << ", " <<  goal.position.z);
	ROS_INFO_STREAM("POSE or: " << goal.orientation.x << ", " << goal.orientation.y << ", " <<  goal.orientation.z << ", " << goal.orientation.w);


	d_arm.setPoseTarget(goal);
	
	moveit_msgs::Constraints constraint;
	moveit_msgs::OrientationConstraint mico_hand_joint_constraint;
	mico_hand_joint_constraint.header.frame_id="base_footprint";
	mico_hand_joint_constraint.link_name = "mico_hand_link";
	mico_hand_joint_constraint.orientation = goal.orientation;
	mico_hand_joint_constraint.absolute_x_axis_tolerance =0.5f;
	mico_hand_joint_constraint.absolute_y_axis_tolerance = 0.5f;
    mico_hand_joint_constraint.absolute_z_axis_tolerance = 0.5f;
	mico_hand_joint_constraint.weight = 0.9;
	constraint.orientation_constraints.push_back(mico_hand_joint_constraint);
	d_arm.setPathConstraints(constraint);


	/*std::vector<double> tolerance_pose(3, 0.03);
	std::vector<double> tolerance_angle(3, 0.03);
	moveit_msgs::Constraints constraint;// = kinematic_constraints::constructGoalConstraints(getEndEffectorLink(), goal, tolerance_pose, tolerance_angle);
	moveit_msgs::JointConstraint mico_shoulder_yaw_joint_constraint;
	mico_shoulder_yaw_joint_constraint.joint_name = "mico_shoulder_yaw_joint";
	mico_shoulder_yaw_joint_constraint.position = -1.57;
	mico_shoulder_yaw_joint_constraint.tolerance_above = 1.25;
	mico_shoulder_yaw_joint_constraint.tolerance_below = 1.25;
	mico_shoulder_yaw_joint_constraint.weight = 1;
	constraint.joint_constraints.push_back(mico_shoulder_yaw_joint_constraint);
	d_arm.setPathConstraints(constraint);*/

	// Plan and execute
    d_arm.asyncMove();
  //  d_is_planning = true;
}

void MoveItInterface::planJointSpaceGoal(std::vector<double> goal_joint_values)
{
	ROS_INFO("Planning joint space goal");

    d_arm.stop();
    d_arm.clearPoseTargets();
	d_arm.setJointValueTarget(goal_joint_values);
/*
	std::vector<double> tolerance_pose(3, 0.03);
	std::vector<double> tolerance_angle(3, 0.03);
	moveit_msgs::Constraints constraint;// = kinematic_constraints::constructGoalConstraints(getEndEffectorLink(), goal_pose, tolerance_pose, tolerance_angle);
	
	moveit_msgs::JointConstraint mico_shoulder_yaw_joint_constraint;
	mico_shoulder_yaw_joint_constraint.joint_name = "mico_shoulder_yaw_joint";
	mico_shoulder_yaw_joint_constraint.position = 4.46804289;
	mico_shoulder_yaw_joint_constraint.tolerance_above = 0.977384381;
	mico_shoulder_yaw_joint_constraint.tolerance_below = 0.977384381;
	mico_shoulder_yaw_joint_constraint.weight = 0.9;
	constraint.joint_constraints.push_back(mico_shoulder_yaw_joint_constraint);
	d_arm.setPathConstraints(constraint);
*/

	// Plan and execute
	d_arm.asyncMove();
	d_is_planning = true;
}

void MoveItInterface::planXYZQuaternionGoal(double pose_x, double pose_y, double pose_z, double orientation_x, double orientation_y, double orientation_z, double orientation_w)
{
	geometry_msgs::Pose pose;
	pose.position.x = pose_x;
	pose.position.y = pose_y;
	pose.position.z = pose_z;
	pose.orientation.x = orientation_x;
	pose.orientation.y = orientation_y;
	pose.orientation.z = orientation_z;
	pose.orientation.w = orientation_w;

	planPoseGoal(pose);
}

void MoveItInterface::planXYZRPYGoal(double pose_x, double pose_y, double pose_z, double roll, double pitch, double yaw)
{
	ROS_INFO("Planning XYZ-RPY goal");

	//ROS_INFO_STREAM("JOINT_TOLERANCE: " << d_arm.getGoalJointTolerance() << ", POSITION_TOLERANCE: " << d_arm.getGoalPositionTolerance() << ", ORIENTATION_TOLERANCE: " << d_arm.getGoalOrientationTolerance());

	// Set target pose
    d_arm.stop();
    d_arm.clearPoseTargets();


    geometry_msgs::Pose pose;
    pose.position.x = pose_x;
	pose.position.y = pose_y;
	pose.position.z = pose_z;
	
	tf::Quaternion q;
	q.setEuler(yaw, pitch, roll);
	geometry_msgs::Quaternion odom_quat;
	tf::quaternionTFToMsg(q, odom_quat);
	
//	pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
	pose.orientation = odom_quat;

//	d_arm.setEndEffectorLink("mico_finger_1_link");
//	d_arm.setPoseReferenceFrame("/mico_shoulder_link");	
//	d_arm.setStartStateToCurrentState();

//	d_arm.setPoseReferenceFrame("mico_base_link");
    
//	d_arm.setPositionTarget(pose_x, pose_y, pose_z);
//	d_arm.setRPYTarget(roll, pitch, yaw);


	robot_state::RobotState start_state(*d_arm.getCurrentState());
	geometry_msgs::Pose start_pose2;
	start_pose2.orientation.w = 1.0;

	start_pose2.position.x = 0.28406;
	start_pose2.position.y = 0.069024;
	start_pose2.position.z = 1.2056;
	const robot_state::JointModelGroup *joint_model_group =
	start_state.getJointModelGroup(d_arm.getName());
	start_state.setFromIK(joint_model_group, start_pose2);
//	d_arm.setStartState(start_state);


	std::vector<geometry_msgs::Pose> waypoints;
	geometry_msgs::Pose target_pose3 = start_pose2;
	target_pose3.position.x += 0.05;
	target_pose3.position.z += 0.05;
	waypoints.push_back(target_pose3); // up and out
	target_pose3.position.y -= 0.05;
	waypoints.push_back(target_pose3); // left
	target_pose3.position.z -= 0.05;
	target_pose3.position.y += 0.05;
	target_pose3.position.x -= 0.05;
	waypoints.push_back(target_pose3); // down and right (back to start)

	moveit_msgs::RobotTrajectory trajectory_msg;
   
    d_arm.setPlanningTime(10.0);
    d_arm.setPlannerId("mico_arm[PRMstarkConfigDefault]"); // allows to set different planner
    double fraction = d_arm.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.00,   // jump_threshold
                                               trajectory_msg, false);

    ROS_INFO_STREAM("Path results: " << fraction);
	moveit::planning_interface::MoveGroup::Plan plan;
    plan.trajectory_ = trajectory_msg;
//    d_arm.execute(plan); 
		
//	ROS_INFO_STREAM("x,y,z: " << pose_x << ", " << pose_y << ", " << pose_z);
//	ROS_INFO_STREAM("r,p,y: " << roll << ", " << pitch << ", " << yaw);
	ROS_INFO_STREAM("PLANNING FRAME: " << d_arm.getPlanningFrame() << ", " << d_arm.getPoseReferenceFrame() << ", " << d_arm.getEndEffectorLink());
	

//	planPoseGoal(pose);
	// Plan and execute
//	d_arm.asyncMove();
//    d_is_planning = true;
}

void MoveItInterface::planDangerousXYZQuaternionGoal(double pose_x, double pose_y, double pose_z, double orientation_x, double orientation_y, double orientation_z, double orientation_w)
{
	ROS_INFO("Sending dangerous goal");

	// Create goal
	jaco_msgs::ArmPoseGoal goal;
	goal.pose.header.frame_id = "/jaco_api_origin";
	goal.pose.pose.position.x = pose_x;
	goal.pose.pose.position.y = pose_y;
	goal.pose.pose.position.z = pose_z;
	goal.pose.pose.orientation.x = orientation_x;
	goal.pose.pose.orientation.y = orientation_y;
	goal.pose.pose.orientation.z = orientation_z;
	goal.pose.pose.orientation.w = orientation_w;

	// Send goal
	d_jaco_dangerous_pose_action_client.sendGoal(goal);

	d_is_planning = true;
}

void MoveItInterface::planDangerousJointAnglesGoal(std::vector<double> goal_joint_values)
{
	ROS_INFO("Sending dangerous joint angles goal");

	// Create goal
	jaco_msgs::ArmJointAnglesGoal goal;
	goal.angles.joint1 = goal_joint_values[0];
    goal.angles.joint2 = goal_joint_values[1];
    goal.angles.joint3 = goal_joint_values[2];
    goal.angles.joint4 = goal_joint_values[3];
    goal.angles.joint5 = goal_joint_values[4];
    goal.angles.joint6 = goal_joint_values[5];

	// Send goal
	d_jaco_dangerous_joint_angles_action_client.sendGoal(goal);

	//d_is_planning = true;
}

void MoveItInterface::planPredefinedGoal(const std::string& goal_name)
{
	std::string goal_name_lower = goal_name;
	std::transform(goal_name_lower.begin(), goal_name_lower.end(), goal_name_lower.begin(), ::tolower);

	// Check if goal exists
	if (d_predefined_positions.count(goal_name_lower))
	{
		ROS_INFO_STREAM("Planning for predefined goal '" << goal_name << "'");
		planJointSpaceGoal(d_predefined_positions.at(goal_name_lower));
		return;
	}
	else
		ROS_WARN_STREAM("Goal '" << goal_name << "' does not exist. Planning failed");
}

void MoveItInterface::simpleMoveGrab(double x, double y, double z)
{
	jaco_msgs::ArmPoseGoal goal;
	goal.pose.header.frame_id = "simpleMoveGrab"; // use it to filter this out in the action client
	goal.pose.pose.position.x = x; // the distance thats needs to be added to the current position
	goal.pose.pose.position.y = y; // the distance thats needs to be added to the current position
	goal.pose.pose.position.z = z; // the distance thats needs to be added to the current position

	d_jaco_dangerous_pose_action_client.sendGoal(goal);
}

void MoveItInterface::openFingers()
{
	ROS_INFO("Opening fingers");
	customFingerPosition(0);
}

void MoveItInterface::closeFingers()
{
	ROS_INFO("Closing fingers");
	customFingerPosition(6000);
}

void MoveItInterface::customFingerPosition(int location)
{
	ROS_INFO_STREAM("Placing fingers at position " << location);

	// Create goal
	jaco_msgs::SetFingersPositionGoal goal;
	goal.fingers.finger1 = location;
    goal.fingers.finger2 = location;
    goal.fingers.finger3 = 0.0f;

	// Send goal
	d_jaco_finger_action_client.sendGoal(goal);

	d_is_planning = true;
}

void MoveItInterface::cancelGoal()
{
	ROS_INFO("Cancelling goal");

	// Reset
    d_execution_result = 99;
    d_dangerous_execution_result = 99;
    d_finger_result = -99;
	d_is_planning = false;
	d_done_moving = true;

	// Restart arm
	stopArm();
	startArm();
}