#include "moveit_interface.ih"

std::string MoveItInterface::getPlanningFrame()
{
	return d_arm.getPlanningFrame();
}

std::string MoveItInterface::getEndEffectorLink()
{
	return d_arm.getEndEffectorLink();
}

std::vector<double> MoveItInterface::getCurrentJointValues()
{
	return d_joint_state.position;
}
