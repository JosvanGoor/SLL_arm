#ifndef H_CLUSTEROBJECTS
#define H_CLUSTEROBJECTS

#include "../utils.h"

class ClusterObjects
{
	private:
		EuclideanClusterExtraction<Point> ec;	

	public:
		ClusterObjects();
		ClusterObjects(double, size_t, size_t);

		void tolerance(double tolerance);
		void maxSize(size_t max_size);
		void minSize(size_t min_size);

		double tolerance();
		size_t maxSize();
		size_t minSize();

		vector<pcl::PointIndices> cluster(PCLPointCloudPtr cloud);


};

inline void ClusterObjects::tolerance(double tolerance)
{
	ec.setClusterTolerance(tolerance);       // in meters
}

inline void ClusterObjects::maxSize(size_t max_size)
{
	ec.setMaxClusterSize(max_size);      // Maximal points that must belong to a cluster
}

inline void ClusterObjects::minSize(size_t min_size)
{
	ec.setMinClusterSize(min_size);      // Maximal points that must belong to a cluster
}

inline double ClusterObjects::tolerance()
{
	return ec.getClusterTolerance();
}

inline size_t ClusterObjects::maxSize()
{
	return ec.getMaxClusterSize();      // Maximal points that must belong to a cluster
}

inline size_t ClusterObjects::minSize()
{
	return ec.getMinClusterSize();      // Maximal points that must belong to a cluster
}

#endif
