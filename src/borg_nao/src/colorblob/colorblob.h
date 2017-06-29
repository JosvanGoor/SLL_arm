#ifndef INCLUDED_COLORBLOB_H
#define INCLUDED_COLORBLOB_H

#include <ros/ros.h>
#include <string>
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>

/**
 * Description
 * 
 * @author Your Name <email@host.com>
 */
class Colorblob
{
    public:
        Colorblob();
	void run();
	static void onMouse(int event, int x, int y, int, void *param);

    private:
	ros::NodeHandle d_nh;
	
	//Subscriber
	ros::Subscriber d_image_sub;

	//Services
	ros::ServiceClient d_memory_client;

	//Objects
	struct color_object
	{
		std::string name;
		int min_size;
		cv::Scalar upper;
		cv::Scalar lower;
	};
	std::vector<color_object> d_objects;

	//Scalars
	cv::Scalar d_ball_upper, d_ball_lower;
	//cv::Mat files. Different images
	cv_bridge::CvImagePtr d_cv_ptr;
	cv::Mat d_HSV_image;
	cv::Mat d_filtered_image;
	cv::Mat d_object_image;	
	
	//GUI variables and states
	bool d_VISUAL_MODE;
	int d_visual_state;
	enum VISUAL_STATE {
		RGB,
		HSV,
		FILTER,
		OBJECT,
		NONE
	};
	
	void newImageCB(const sensor_msgs::ImageConstPtr& msg);
	void visualMode();
	void updateVisualState();
	void writeToMemory(std::vector<cv::Rect>& obs, std::string name);
	void findBlobs(color_object& object);
	void sortObservations(std::vector<cv::Rect>& obs);
};

        
#endif
