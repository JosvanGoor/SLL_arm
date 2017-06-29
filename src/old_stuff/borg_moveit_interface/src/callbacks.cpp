#include "moveit_interface.ih"

void MoveItInterface::armStateCallback(const sensor_msgs::JointState::ConstPtr& arm_state)
{
    d_joint_state = *arm_state;
    d_got_data = true;
}

void MoveItInterface::executionResultsCallback(const control_msgs::FollowJointTrajectoryActionResult::ConstPtr& results)
{
    d_execution_result = results->result.error_code;
 
    // Sometimes a result is received slightly before the goal was actually reached.
    // Planning another goal when a motion was not finished yet will result in chaos
}

void MoveItInterface::fingerResultsCallback(const jaco_msgs::SetFingersPositionActionResult::ConstPtr& results)
{
	d_finger_result = results->status.status;
}

void MoveItInterface::dangerousPoseResultsCallback(const jaco_msgs::ArmPoseActionResult::ConstPtr& results)
{
	d_dangerous_execution_result = results->status.status;
}

void MoveItInterface::dangerousJointAnglesResultsCallback(const jaco_msgs::ArmJointAnglesActionResult::ConstPtr& results)
{
	d_dangerous_execution_result = results->status.status;
}