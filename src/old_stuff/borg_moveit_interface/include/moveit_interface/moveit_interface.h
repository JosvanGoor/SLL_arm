#ifndef _MOVEIT_INTERFACE_H
#define _MOVEIT_INTERFACE_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/Pose.h>
//#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <vector>

#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <jaco_msgs/SetFingersPositionAction.h>
#include <jaco_msgs/ArmPoseAction.h>
#include <jaco_msgs/ArmJointAnglesAction.h>

#include <tf/transform_datatypes.h>
 
class MoveItInterface
{
    
private:
    ros::NodeHandle d_nh;

    // Subscribers and service clients
    ros::Subscriber d_arm_state_subscriber;
    ros::Subscriber d_execution_result_subscriber;
    ros::ServiceClient d_ik_compute_service;
    ros::ServiceClient d_jaco_stop_service;
    ros::ServiceClient d_jaco_start_service;
    actionlib::SimpleActionClient<jaco_msgs::SetFingersPositionAction> d_jaco_finger_action_client;
    ros::Subscriber d_finger_result_subscriber;
    actionlib::SimpleActionClient<jaco_msgs::ArmPoseAction> d_jaco_dangerous_pose_action_client;
    ros::Subscriber d_dangerous_pose_result_subscriber;
    actionlib::SimpleActionClient<jaco_msgs::ArmJointAnglesAction> d_jaco_dangerous_joint_angles_action_client;
    ros::Subscriber d_jaco_dangerous_joint_angles_result_subscriber;
    ros::ServiceClient d_memory_writer;
    ros::ServiceClient d_memory_reader;
    
    // Params
    std::string d_group_name;
    std::string d_joint_state_topic;
    std::string d_trajectory_result_topic;
    std::string d_finger_goal_action_topic;
    std::string d_finger_result_topic;
    std::string d_dangerous_pose_action_topic;
    std::string d_dangerous_pose_result_topic;
    std::string d_dangerous_joint_angles_action_topic;
    std::string d_dangerous_joint_angles_result_topic;
    double d_planning_timeout;
    
    // Move_group
    moveit::planning_interface::MoveGroup d_arm;
    moveit::planning_interface::PlanningSceneInterface d_planning_scene_interface;

    // Data
    std::map<std::string, std::vector<double> > d_predefined_positions;
    bool d_got_data;
    sensor_msgs::JointState d_joint_state;
    ros::Time d_last_time_memory_emergency_received;
    ros::Time d_last_time_memory_goal_received;
    ros::Time d_last_time_memory_cancel_goal_received;
    bool d_arm_stopped;

    // Planning results and states
    bool d_done_moving;
    bool d_is_planning;
    int d_execution_result;
    int d_dangerous_execution_result;
    int d_finger_result;
 
public:
    MoveItInterface(ros::NodeHandle &nh, 
        const std::string& group_name, 
        const std::string& joint_state_topic, 
        const std::string& trajectory_result_topic, 
        const std::string& finger_goal_action_topic, 
        const std::string& finger_result_topic,
        const std::string& dangerous_pose_action_topic,
        const std::string& dangerous_pose_result_topic,
        const std::string& dangerous_joint_angles_action_topic,
        const std::string& dangerous_joint_angles_result_topic);
    void initHardCodedPositions();

    // Update
    void update();

    // Memory functions
    void readFromMemory();
    void readEmergencyStateFromMemory();
    void readXYZQuaternionGoalFromMemory();
    void readXYZRPYGoalFromMemory();
    void readDangerousXYZQuaternionGoalFromMemory();
    void readDangerousJointAnglesGoalFromMemory();
    void readPredefinedGoalFromMemory();
    void readFingerGoalFromMemory();
    void readCancelGoalFromMemory();
    void readSimpleGrabFromMemory();
    void checkEmergencyState();
    void sendFeedbackToMemory(bool success);
    void sendGoalFeedbackToMemory();
    void sendDangerousGoalFeedbackToMemory();
    void sendFingerFeedbackToMemory();

    // Getters from move_group
    std::string getPlanningFrame();
    std::string getEndEffectorLink();
    std::vector<double> getCurrentJointValues();

    // Calculation functions
    std::vector<double> calculateIK(geometry_msgs::Pose pose);

    // Plan/move functions
    void planPoseGoal(geometry_msgs::Pose goal);
    void planJointSpaceGoal(std::vector<double> goal_joint_values);
    void planXYZQuaternionGoal(double pose_x, double pose_y, double pose_z, double orientation_x, double orientation_y, double orientation_z, double orientation_w);
    void planXYZRPYGoal(double pose_x, double pose_y, double pose_z, double roll, double pitch, double yaw);
    void planDangerousXYZQuaternionGoal(double pose_x, double pose_y, double pose_z, double orientation_x, double orientation_y, double orientation_z, double orientation_w);
    void planDangerousJointAnglesGoal(std::vector<double> goal_joint_values);
    void planPredefinedGoal(const std::string& goal_name);
    void simpleMoveGrab(double x, double y, double z);
    void openFingers();
    void closeFingers();
    void customFingerPosition(int location);
    void cancelGoal();

    // Callback functions
    void armStateCallback(const sensor_msgs::JointState::ConstPtr& arm_state);
 	void executionResultsCallback(const control_msgs::FollowJointTrajectoryActionResult::ConstPtr& results);
    void fingerResultsCallback(const jaco_msgs::SetFingersPositionActionResult::ConstPtr& results);
    void dangerousPoseResultsCallback(const jaco_msgs::ArmPoseActionResult::ConstPtr& results);
    void dangerousJointAnglesResultsCallback(const jaco_msgs::ArmJointAnglesActionResult::ConstPtr& results);

 	// JACO functions
 	void stopArm();
 	void startArm();
};
 
#endif
