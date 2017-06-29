#ifndef H_SIFT
#define H_SIFT

#include "../utils.h"

struct Object
{
	string objectName; 
	cv::Mat image;
	vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	bool largeObject; // if true object is large, else object is small;
};



class Sift
{
	vector<Object> objects;
	
	public: 
		Sift();
		void init();
		set<string> match(cv::Mat image, bool largeObject);
		float getVolume(string path);
		float getVolume(PCLPointCloud &cloud);

}; 

#endif
