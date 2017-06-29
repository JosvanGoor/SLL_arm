#include "simpleFeature.h"

void SimpleFeature::makeImage(PCLPointCloud &cloud)
{
	d_unmodCloud = cloud;
	int width = 480;
	int height = 640;
	cv::Mat newImage(width, height, CV_8UC3);
	
	int count = 0;
		
	for (size_t idx = 0; idx < width; ++idx)
	{
		for (size_t idy = 0; idy < height; ++idy)
		{
			int b =	cloud.points[count].b;
			int g =	cloud.points[count].g;
			int r =	cloud.points[count].r;
			newImage.at<cv::Vec3b>(idx, idy)[0] = (unsigned char)b;
			newImage.at<cv::Vec3b>(idx, idy)[1] = (unsigned char)g;
			newImage.at<cv::Vec3b>(idx, idy)[2] = (unsigned char)r;					
			++count;	
		}
	}
	
	d_cloudImage = newImage.clone();
}