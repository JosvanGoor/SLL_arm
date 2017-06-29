#ifndef H_OBJECTDETECTION
#define H_OBJECTDETECTION

#include "../utils.h"

#include "../planarRemover/planarRemover.h"
#include "../clusterObjects/clusterObjects.h"
#include "../simpleFeature/simpleFeature.h"
#include "../sift/sift.h"



struct DetectedObject
{
	string objectName;
	float x;
	float y;
	float z;
	float h; // height of object
};

class ObjectDetection
{
	// Memory service client
	ros::ServiceClient client_reader;
	ros::ServiceClient client_writer;

	//  Publishing a cloud 
	NodeHandle nh;
	Publisher cloud_pub;
		
	// modules
	PlanarRemover planarRemover;
	ClusterObjects clusterObjects;
	SimpleFeature simpleFeature;
	Sift sift;	
	
	PCLPointCloud objectCloud;	
	vector<SimpleFeature> d_simpleFeatures;

	size_t d_numberOfSamples;
	size_t d_currentSampleNr;
	
	public:
		ObjectDetection();                                  
		void pcCallBack(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);
		void init();

	private:
		bool run();
		void writeToMemory(vector<DetectedObject> objects);
		void writeStop();
		
};

#endif
