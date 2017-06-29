#include "moveit_interface.ih"

MoveItInterface::MoveItInterface(ros::NodeHandle &nh, 
  const std::string& group_name, 
  const std::string& joint_state_topic, 
  const std::string& trajectory_result_topic, 
  const std::string& finger_goal_action_topic, 
  const std::string& finger_result_topic,
  const std::string& dangerous_pose_action_topic,
  const std::string& dangerous_pose_result_topic,
  const std::string& dangerous_joint_angles_action_topic,
  const std::string& dangerous_joint_angles_result_topic)
    : d_nh(nh),
      d_jaco_finger_action_client(finger_goal_action_topic),
      d_jaco_dangerous_pose_action_client(dangerous_pose_action_topic),
      d_jaco_dangerous_joint_angles_action_client(dangerous_joint_angles_action_topic),
      d_group_name(group_name),
      d_joint_state_topic(joint_state_topic),
      d_trajectory_result_topic(trajectory_result_topic),
      d_finger_goal_action_topic(finger_goal_action_topic),
      d_finger_result_topic(finger_result_topic),
      d_dangerous_pose_action_topic(dangerous_pose_action_topic),
      d_dangerous_pose_result_topic(dangerous_pose_result_topic),
      d_dangerous_joint_angles_action_topic(dangerous_joint_angles_action_topic),
      d_dangerous_joint_angles_result_topic(dangerous_joint_angles_result_topic),
      d_planning_timeout(60),
      d_arm(group_name),
      d_got_data(false),
      d_last_time_memory_emergency_received(0),
      d_last_time_memory_goal_received(0),
      d_arm_stopped(false),
      d_done_moving(false),
      d_is_planning(false),
      d_execution_result(99),
      d_dangerous_execution_result(99),
      d_finger_result(-99)
{    
    // Set configuration
    d_arm.setPlanningTime(d_planning_timeout);
    d_arm.setGoalJointTolerance(0.005);
    d_arm.setGoalPositionTolerance(0.005);
    d_arm.setGoalOrientationTolerance(0.005);
    d_arm.allowReplanning(false);

    //d_arm.setPlannerId("mico_arm[RRTConnectkConfigDefault]"); // allows to set different planner
   // d_arm.setPlannerId("mico_arm[PRMstarkConfigDefault]"); // allows to set different planner
   // d_arm.setPlannerId("mico_arm[RRTstarkConfigDefault]"); // allows to set different planner
    
    // Init hardcoded positions
    initHardCodedPositions();
    
    // Dynamic reconfigure
    system("rosrun dynamic_reconfigure dynparam set /move_group/trajectory_execution/ allowed_execution_duration_scaling \"10\"");

    // Subscribe to joint states
    d_arm_state_subscriber = d_nh.subscribe<sensor_msgs::JointState>(
        d_joint_state_topic, 1, &MoveItInterface::armStateCallback, this);
        
    // Subscribe to execution results
    d_execution_result_subscriber = d_nh.subscribe<control_msgs::FollowJointTrajectoryActionResult>(
        d_trajectory_result_topic, 1, &MoveItInterface::executionResultsCallback, this);
        
    // Create client for ik_compute service
    d_ik_compute_service = d_nh.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
    ROS_INFO("compute_ik service ready");
    
    // Subscribe to JACO stop and start service
    d_jaco_stop_service = d_nh.serviceClient<jaco_msgs::Stop>("/jaco/stop");
    d_jaco_start_service = d_nh.serviceClient<jaco_msgs::Start>("/jaco/start");

    // Wait for jaco finger action client and subscribe to the results topic
  //  d_jaco_finger_action_client.waitForServer();
    
    ROS_INFO("fingers (not) ready!");
    
    d_finger_result_subscriber = d_nh.subscribe<jaco_msgs::SetFingersPositionActionResult>(
        d_finger_result_topic, 1, &MoveItInterface::fingerResultsCallback, this);

    // Wait for jaco pose action client and subscribe to the results topic
    d_jaco_dangerous_pose_action_client.waitForServer();
    
    ROS_INFO("dangerous pose ready!");
    d_dangerous_pose_result_subscriber = d_nh.subscribe<jaco_msgs::ArmPoseActionResult>(
        d_dangerous_pose_result_topic, 1, &MoveItInterface::dangerousPoseResultsCallback, this);

    // Wait for jaco joint angles action client and subscribe to the results topic
 //   d_jaco_dangerous_joint_angles_action_client.waitForServer();
    
    ROS_INFO("dangerous angle (not)ready!");
    d_jaco_dangerous_joint_angles_result_subscriber = d_nh.subscribe<jaco_msgs::ArmJointAnglesActionResult>(
        d_dangerous_joint_angles_result_topic, 1, &MoveItInterface::dangerousJointAnglesResultsCallback, this);

    // Create client for borg memory service
    d_memory_reader = d_nh.serviceClient<borg_pioneer::MemoryReadSrv>("/memory_read");
    d_memory_writer = d_nh.serviceClient<borg_pioneer::MemorySrv>("/memory");

    ROS_INFO("MoveIt! interface is ready, you can start planning now!!");
}
