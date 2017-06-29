#include <ros/ros.h>
#include <micoArm.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mico_controller");
	ros::NodeHandle nh;

	MicoArm mico(nh);

	ros::spin();
}
