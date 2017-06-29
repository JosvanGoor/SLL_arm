#include <ros/ros.h>
#include <headSpinner.h>


double degreeToRad(double deg)
{
	return deg * M_PI / 180.0;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "head_spin");
	ros::NodeHandle nh;
	HeadSpinner headSpinner(nh);

	ros::spin();

}




