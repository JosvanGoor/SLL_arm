#include "objectRecognizer.ih"

/*
 * Contains:
 * colorClusters: colors all clusters of an inputcloud either spread over grey values of colored values
 * colorcluster: overloaded function to change the color of a cluster to a given greyscale value
 * colorcluster: overloaded function to change the color of a cluster to a given RGB value
 */

//colors all clusters of an inputcloud either spread over grey values of colored values
void ObjectRecognizer::colorClusters(const vector<PointIndices> clusters, PCLPointCloudPtr input, bool grey)
{	
	//if we desire grayscale clouds...
	if (grey)
	{
		//Set each clusters' color to a scaled grey value
	    for (size_t idx = 0; idx != clusters.size(); ++idx)
	    	colorCluster(clusters[idx], input, (255 / clusters.size()) * (idx + 1));
	} else
	{
		//Otherwise set each clusters' color to an RGB value scaled from red to blue.
		//irrelevant(?): Green is kept to zero for debugging possibilities
	    for (size_t idx = 0; idx != clusters.size(); ++idx)
	    	colorCluster(clusters[idx], input, (255 / clusters.size()) * (idx + 1), 0, 255 - (255 / clusters.size()) * (idx + 1));		
	}
}

//overloaded function to change the color of a cluster to a given greyscale value
void ObjectRecognizer::colorCluster(const PointIndices cluster, PCLPointCloudPtr input, size_t color)
{
	//Set the R, G and B values of each point in a cluster to the given color of grey.
	vector<int> my_indices = cluster.indices;
	for (size_t idx = 0; idx != my_indices.size(); ++idx)
	{
		input->points[my_indices[idx]].r = color;
		input->points[my_indices[idx]].g = color;
		input->points[my_indices[idx]].b = color;
	}
}

//overloaded function to change the color of a cluster to a given RGB value
void ObjectRecognizer::colorCluster(const PointIndices cluster, PCLPointCloudPtr input, size_t r, size_t g, size_t b)
{
	//Set the R, G and B values of each point in a cluster to their provided values.
	vector<int> my_indices = cluster.indices;
	for (size_t idx = 0; idx != my_indices.size(); ++idx)
	{
		input->points[my_indices[idx]].r = r;
		input->points[my_indices[idx]].g = g;
		input->points[my_indices[idx]].b = b;
	}
}
