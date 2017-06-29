#include "utils.h"

#include "objectRecognizer/objectRecognizer.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camtest");
    ros::NodeHandle n;

    ros::Rate r(10);
    ObjectRecognizer objRec;
    string input_topic;
	tf::TransformListener temp_listener;
	objRec.tfTransformer = &temp_listener;
	bool runexperiment = false;
    param::param<std::string>("~input_topic", input_topic, "front_xtion/depth_registered/points");

	if (runexperiment)
		objRec.runExperiment();
	else
	{
	    ros::Subscriber depth_sub = n.subscribe<sensor_msgs::PointCloud2>("/alice_pointcloud/objectsCloud", 1, &ObjectRecognizer::rosrun, &objRec);
	    //ros::Subscriber depth_sub = n.subscribe<sensor_msgs::PointCloud2>("camera/depth_registered/points", 1, &ObjectRecognizer::recordExperiment, &objRec);

	    while(ros::ok())
	    {
			ros::spinOnce();
			r.sleep();
	    }
    }
	return 0;
}
