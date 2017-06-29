#include "moveit_interface.ih"

void MoveItInterface::update()
{
	// Check for memory updated
	readFromMemory();
    checkEmergencyState();

    // Check if we have any data
    if (!d_got_data)
        return;

    // Next update we want to have new data
    d_got_data = false;

    // Check for moveit feedback
    if (d_execution_result != 99)
	{
		ROS_INFO("Goal reached");
        sendGoalFeedbackToMemory();
        d_is_planning = false;
        d_execution_result = 99;
	}

    // Check for dangerous arm pose feedback
    if (d_dangerous_execution_result != 99)
    {
        ROS_INFO("Goal reached");
        sendDangerousGoalFeedbackToMemory();
        d_is_planning = false;
        d_dangerous_execution_result = 99;
    }

    // Check for finger feedback
    if (d_finger_result != -99)
    {
        ROS_INFO("Fingers goal reached");
        sendFingerFeedbackToMemory();
        d_is_planning = false;
        d_finger_result = -99;
    }
}

void MoveItInterface::checkEmergencyState()
{
    // See how long it has been that we received emergency state
    ros::Duration d = ros::Time::now() - d_last_time_memory_emergency_received;

    // Stop arm when in an emergency
    if (d.toSec() < 0.25 && !d_arm_stopped)
    {
        stopArm();
        d_arm_stopped = true;
    }
    
    if (d.toSec() > 0.5 && d_arm_stopped)
    {
        startArm();
        d_arm_stopped = false;
    }
}