#include "moveit_interface.ih"

void MoveItInterface::stopArm()
{
	jaco_msgs::Stop srv;
	if (d_jaco_stop_service.call(srv))
		ROS_INFO("Robot arm stopped");
}

void MoveItInterface::startArm()
{
	jaco_msgs::Start srv;
	if (d_jaco_start_service.call(srv))
		ROS_INFO("Robot arm started");
}
