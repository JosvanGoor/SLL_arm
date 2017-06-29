#include "colorblob.ih"

void Colorblob::findBlobs(Colorblob::color_object& object)
{
	cv::Mat mask;
	vector<vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;
	vector<cv::Rect> observations;
	
	//Param shit..
	cv::inRange(d_HSV_image, object.lower, object.upper , mask);

	cv::bitwise_and(d_HSV_image, d_HSV_image, d_filtered_image, mask);

	cv::findContours(mask, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	for(vector<vector<cv::Point> >::iterator contour = contours.begin(); contour != contours.end(); ++contour)
	{
		//Minimal size blob == 10
		if  (cv::contourArea(*contour) > object.min_size)
		{
			cv::Rect boundRect = boundingRect(*contour);
			//std::cout <<  "colorblob name: " << object.name << std::endl;
			cv::Scalar col = cv::Scalar(255,255,255);
			if(object.name == "colorblob_red") col = cv::Scalar(0, 0, 255);
			if(object.name == "colorblob_yellow") col = cv::Scalar(0, 255, 255);
			if(object.name == "colorblob_blue") col = cv::Scalar(255, 0, 0);
			
			cv::rectangle(d_object_image, boundRect.tl(), boundRect.br(), col, 1, 8, 0);			
			
			//Add observation to memory vector
			observations.push_back(boundRect);
		}
	}

	if (observations.size() > 0)
	{
		writeToMemory(observations, object.name);
	}
}
