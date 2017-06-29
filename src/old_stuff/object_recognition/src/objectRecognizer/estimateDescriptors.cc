#include "objectRecognizer.ih"

/*
 * Contains:
 * estimateDescriptors: estimate the descriptorvalues for all keypoints in each cluster
 * getSHOTdescriptors: compute the SHOT descriptors for all keypoints in a cluster
 */

//estimate the descriptorvalues for all keypoints in each cluster
void ObjectRecognizer::estimateDescriptors(vector<PCLPointCloudPtr> clusterPCLs, vector<vector<int> > keypoints, vector<PointCloud<Normal>::Ptr> normals )
{
	//clear any eventual information on descriptors that we had
    d_inputDescriptors.clear();

	//For all object clusters...
	for (size_t idx = 0; idx != clusterPCLs.size(); ++idx)
	{
		//compute the SHOT descriptors
		PointCloud<descriptorType>::Ptr descriptorClust(new PointCloud<descriptorType>);
		getSHOTDescriptors(clusterPCLs[idx], keypoints[idx], normals[idx], *descriptorClust);

		//and store them
		d_inputDescriptors.push_back(descriptorClust);
	}

}

//compute the SHOT descriptors for all keypoints in a cluster
void ObjectRecognizer::getSHOTDescriptors(PCLPointCloudPtr inputImg, vector<int> keypoints, PointCloud<Normal>::Ptr normals, PointCloud<descriptorType> &output)
{
	//Set keypoints
	PointIndices::Ptr keyIndices(new PointIndices);
	keyIndices->indices = keypoints;

	//create searchtree from the inputcloud
	search::KdTree<Point>::Ptr tree (new search::KdTree<Point>);
	tree->setInputCloud(inputImg);

	//Set parameters for SHOT estimation
	SHOTEstimation<Point, Normal, descriptorType> shot;
	shot.setSearchMethod (tree); //kdtree
	shot.setIndices (keyIndices); //keypoints
	shot.setInputCloud (inputImg); //input
	shot.setInputNormals(normals);//normals
	shot.setRadiusSearch (0.06); //support

	//compute the actual descriptors
	shot.compute (output);
}
