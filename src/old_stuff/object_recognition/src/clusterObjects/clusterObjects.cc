#include "clusterObjects.ih"

ClusterObjects::ClusterObjects()
{
	ec.setClusterTolerance(0.02);       // in meters
	ec.setMinClusterSize(100);         // Minimal points that must belong to a cluster
	ec.setMaxClusterSize(10000);      // Maximal points that must belong to a cluster
} 

ClusterObjects::ClusterObjects(double tol, size_t min_size, size_t max_size)
{
	tolerance(tol);
	minSize(min_size);
	maxSize(max_size);	
}
