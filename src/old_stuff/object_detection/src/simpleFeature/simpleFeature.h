#ifndef H_SIMPLEFEATURE
#define H_SIMPLEFEATURE

#include "../utils.h"

struct SimpleFeatures
{
	string objectName;
	float minDistance;
	float meanDistance;
	float maxDistance;
	int cloudSize;
	Point meanPoint;
	PCLPointCloud objectCloud;
	pcl::PointCloud<pcl::Normal> objectNormals;
	vector<pcl::Normal> normalFeatures;	

	float volume; // in cm
};

struct ObjectWindow
{
	size_t xLeft;
	size_t xRight;
	size_t yTop;
	size_t yBottom;
	Point minPt;
	Point maxPt;
};

class SimpleFeature
{
	Publisher publisher;
	NodeHandle nh;
	vector<SimpleFeatures> d_simpleFeatures;
	PCLPointCloud d_objectCloud;
	PCLPointCloud d_unmodCloud;
	cv::Mat d_cloudImage;
	
	public:
		SimpleFeature() { init(); publisher = nh.advertise<PCLPointCloud>("simpleFeature/output", 1); }; // inline	
		void makeImage(PCLPointCloud &cloud);
		void setObjectCloud(PCLPointCloud &cloud);
		void match();
		void showImage();
		ObjectWindow getObjectWindow();
		cv::Mat getImage();
	
	private:
		void init();
		void initNormalFeature();
		void initVolumeFeature();
};

#endif
