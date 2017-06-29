#include "objectRecognizer.ih"

/*
 * Contains: 
 * cleanUpImage: Removes all points from a Pointcloud that do not belong to a given cluster
 */

 //Removes all points from a Pointcloud that do not belong to a given cluster
void ObjectRecognizer::cleanUpImage(PCLPointCloudPtr inputCloud, vector<PointIndices> clusterIndices)
{
	//create a buffer cloud and an empty cloud to store the temporary results
	PCLPointCloudPtr tmpCloud(new PCLPointCloud);
	PCLPointCloudPtr outputCloud(new PCLPointCloud);

	//copy all the points that belong to the first cluster
	//this is done explicitly to get the right sizes for the outputCloud
	if (clusterIndices.size())
		copyPointCloud(*inputCloud, clusterIndices[0].indices, *outputCloud);

	//Copy all other clusters to the buffer and add them to the output
	for (size_t idx = 1; idx < clusterIndices.size(); ++idx)
	{
	    copyPointCloud (*inputCloud, clusterIndices[idx].indices, *tmpCloud);
		*outputCloud += *tmpCloud;
	}

	//Overwrite the old and dirty image with the shiny and clean one
	*inputCloud = *outputCloud;
}