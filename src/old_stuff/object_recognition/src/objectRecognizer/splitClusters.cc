#include "objectRecognizer.ih"

/*
Contains: 
splitClusters: splits an inputcloud into a vector of pointclouds per cluster
niceSize: checks whether a cluster's dimensions agree with the database

Given a pointcloud that contains several clusters and a vector of pointindices 
corresponding to those clusters, split up the pointcloud into a vector of 
pointclouds, with each pointcloud in the output vector ('clusters') corresponding
to one cluster. If a cluster has dimensions that do not correspond to any of the 
objects in the database, ignore the cluster
*/

vector<PCLPointCloudPtr> ObjectRecognizer::splitClusters(PCLPointCloudPtr inputCloud, vector<PointIndices> clusterIndices)
{
	//For each element in 'clusterIndices', copy that cluster to a new pointcloud
	//and add it to the output vector
	vector<PCLPointCloudPtr> clusters;

	for(size_t idx = 0; idx != clusterIndices.size(); ++idx)
	{
		PCLPointCloudPtr tmp(new PCLPointCloud);
		copyPointCloud(*inputCloud, clusterIndices[idx].indices, *tmp);

		//Check whether the cluster's dimensions corrsepond to any object in the
		//database
		if (niceSize(tmp))
			clusters.push_back(tmp);
	}

	return clusters;
}

bool ObjectRecognizer::niceSize(PCLPointCloudPtr cluster)
{
	//Compare the dimensions of the inputcloud to the dimensions of each object in 
	//the database. Return 'true' if the dimensions match those of (at least) one 
	// object in the database, return 'false' otherwise
	
	vector<double> extremes;
	extremes = getExtremes(cluster);

	double width = extremes[1] - extremes[0];
	double height = extremes[3] - extremes[2];

	for (db_it it = d_shotDatabase.begin(); it != d_shotDatabase.end(); ++it)
	{

		if (d_dimDatabase[it->first][0] > width  || d_dimDatabase[it->first][1] < width ||
			d_dimDatabase[it->first][2] > height  || d_dimDatabase[it->first][3] < height)
			return true;
	}

	return true;
}