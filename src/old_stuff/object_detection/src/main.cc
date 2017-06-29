#include "utils.h"
#include "objectDetection/objectDetection.h"



int main(int argc, char** argv)
{
	cout << "Started object recognition node... \n";
	
	// ROS initialization stuff
	ros::init(argc, argv, "objectDetection");
	ros::NodeHandle n;
	
	//create object detection
	ObjectDetection objectDetection;
	
	//Subscribe to point cloud data
	ros::Subscriber depth_sub = n.subscribe<sensor_msgs::PointCloud2>("camera/depth_registered/points", 1, &ObjectDetection::pcCallBack, &objectDetection);

	ros::spin();
	
}