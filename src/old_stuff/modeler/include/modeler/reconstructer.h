#ifndef H_RECONSTRUCT
#define H_RECONSTRUCT

#include <modeler/utils.h>

class Reconstructer
{
	// nodehandle 
	ros::NodeHandle d_nh;

	// cloud publisher
	ros::Publisher d_cloud_pub;
	public:
		Reconstructer();
		void reconstruct(vector<PCLPointCloud> vObject);
};

#endif