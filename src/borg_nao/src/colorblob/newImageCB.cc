#include "colorblob.ih"

void Colorblob::newImageCB(const sensor_msgs::ImageConstPtr& msg)
{
	d_cv_ptr = cv_bridge::toCvCopy(msg);

	//Blur image
	cv::blur(d_cv_ptr->image, d_cv_ptr->image, cv::Size(5,5));
	//Convert RGB image to HSV
	cv::cvtColor(d_cv_ptr->image, d_HSV_image, CV_RGB2HSV);

	d_object_image = d_cv_ptr->image.clone();	

	for (size_t idx = 0; idx < d_objects.size(); ++idx) 
		findBlobs(d_objects.at(idx));
	
	if (d_VISUAL_MODE)
		visualMode();

	d_filtered_image.release();
	d_object_image.release();
}
