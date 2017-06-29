#ifndef H_CLUSTEROBJECTS
#define H_CLUSTEROBJECTS

#include "../utils.h"

class ClusterObjects
{
		
	public:
		ClusterObjects(){};
		vector<pcl::PointIndices> cluster(PCLPointCloudPtr cloud);
};

#endif
