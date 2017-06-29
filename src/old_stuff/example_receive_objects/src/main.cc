#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

//opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <objects_2d/objects2d.h>
#include <sensor_msgs/image_encodings.h>

#include <sstream>

// namespaces
using namespace std;
using namespace ros;

void objectsCallBack(objects_2d::objects2d msg)
{
	vector<sensor_msgs::Image> vObjects;
	vObjects = msg.images;

	for (size_t idi = 0; idi < vObjects.size(); ++idi)
	{

		cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(vObjects.at(idi), sensor_msgs::image_encodings::BGR8);
		cv::Mat image = cv_ptr->image;  // this is the image you need for processing

		stringstream ss;
		ss << idi;
		cv::imshow(ss.str().c_str(), image);
	}
	cv::waitKey(100);
}

int main(int argc, char **argv)
{
	// ROS initialization stuff
	ros::init(argc, argv, "example_receive_objects");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe<objects_2d::objects2d>("objects2d", 1, objectsCallBack);

	ros::spin();
}
