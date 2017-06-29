#include "clusterObjects.h"

vector<pcl::PointIndices> ClusterObjects::cluster(PCLPointCloudPtr cloud)
{
	std::vector<pcl::PointIndices> cluster_indices;     // Cluster information is stored here
//	pcl::search::OrganizedNeighbor<Point>::Ptr tree (new pcl::search::OrganizedNeighbor<Point>);  // Creating the KdTree object for the search method of the extraction
//	KdTreePtr<Point> tree (new pcl::search::KdTree<Point>);  // Creating the KdTree object for the search method of the extraction

	pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
//	pcl::search::Octree<Point> tree;
//	KdTreeFLANN<Point>::Ptr tree (new KdTreeFLANN<Point>);
//	search::OrganizedDataIndex<Point>::Ptr tree2 (new search::OrganizedDataIndex<Point>);	
//	search::OrganizedNeighbor<Point>::Ptr treeOrg (new search::OrganizedNeighbor<Point>);

/*
	PCLPointCloudPtr cloud_filtered(new PCLPointCloud);

	pcl::VoxelGrid<Point> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize (0.005f, 0.005f, 0.005f);
	sor.filter(*cloud_filtered);
*/
	tree->setInputCloud(cloud);
	EuclideanClusterExtraction<Point> ec;	
	
	ec.setClusterTolerance(0.02);       // in meters
	ec.setMinClusterSize(100);         // Minimal points that must belong to a cluster
	ec.setMaxClusterSize(500);      // Maximal points that must belong to a cluster
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);
	
	return cluster_indices;
}