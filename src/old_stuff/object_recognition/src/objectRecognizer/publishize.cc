#include "objectRecognizer.ih"

/*
Given a vector of pointclouds and clusterindices, combine them into
one pointcloud that's suitable for visualization in RViz 
*/

PCLPointCloudPtr ObjectRecognizer::publishize(vector<PCLPointCloudPtr> clusterPCLs, vector<vector<int> > indices)
{
	PCLPointCloudPtr publishMe(new PCLPointCloud);

	if (indices.size())
		copyPointCloud(*clusterPCLs[0], indices[0], *publishMe);

	vector<PCLPointCloudPtr> sizinator;

	for (size_t idx = 1; idx < clusterPCLs.size(); ++idx)
	{
		PCLPointCloudPtr tmp(new PCLPointCloud);
		copyPointCloud(*clusterPCLs[idx], indices[idx], *tmp);
		
		sizinator.push_back(tmp);

		*publishMe += *tmp;
	}

	return publishMe;
}