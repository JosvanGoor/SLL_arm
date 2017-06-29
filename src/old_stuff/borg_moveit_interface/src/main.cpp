#include "moveit_interface.ih"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_interface");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(10);
    
    // Get params
    std::string group_name, 
        joint_state_topic, 
        trajectory_result_topic,
        finger_goal_action_topic,
        finger_result_topic,
        dangerous_pose_action_topic,
        dangerous_pose_result_topic,
        dangerous_joint_angles_action_topic,
        dangerous_joint_angles_result_topic;
    n.param<std::string>("group_name", group_name, "mico_arm");
    n.param<std::string>("joint_state_topic", joint_state_topic, "/jaco/joint_state");
    n.param<std::string>("trajectory_result_topic", trajectory_result_topic, "/jaco/trajectory/result");
    n.param<std::string>("finger_goal_action_topic", finger_goal_action_topic, "/jaco/finger_joint_angles");
    n.param<std::string>("finger_result_topic", finger_result_topic, "/jaco/finger_joint_angles/result");
    n.param<std::string>("dangerous_pose_action_topic", dangerous_pose_action_topic, "/jaco/arm_pose");
    n.param<std::string>("dangerous_pose_result_topic", dangerous_pose_result_topic, "/jaco/arm_pose/result");
    n.param<std::string>("dangerous_joint_angles_action_topic", dangerous_joint_angles_action_topic, "/jaco/arm_joint_angles");
    n.param<std::string>("dangerous_joint_angles_result_topic", dangerous_joint_angles_result_topic, "/jaco/arm_joint_angles/result");

    // Create moveit interface instance
    MoveItInterface interface(
        n, 
        group_name, 
        joint_state_topic, 
        trajectory_result_topic, 
        finger_goal_action_topic, 
        finger_result_topic, 
        dangerous_pose_action_topic,
        dangerous_pose_result_topic,
        dangerous_joint_angles_action_topic,
        dangerous_joint_angles_result_topic
    );

    while (ros::ok())
    {
    	interface.update();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
