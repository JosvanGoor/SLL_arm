#include <iostream>
#include <ros/ros.h>

#include <arm_planner/pathPlanner.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "arm_planner");
	ros::NodeHandle node_handle;

	PathPlanner pathPlanner(node_handle);

	ros::spin();
}
