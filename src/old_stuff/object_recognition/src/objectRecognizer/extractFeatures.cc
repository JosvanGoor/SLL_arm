#include "objectRecognizer.ih"

/*
 * Contains:
 * extractFeatures: overloaded function that extracts the required features for a set of clusters
 * extractFeatures: overloaded function that extracts the required features from a single cluster
 */

//overloaded function that extracts the required features for a set of clusters
void ObjectRecognizer::extractFeatures(vector<PCLPointCloudPtr> clusterPCLs)
{
    //find keypoints through uniform sampling
    vector<vector<int> > keypoints;
    vector<PointCloudOut> PCLKeypoints;
    keypoints = getKeypoints(clusterPCLs, PCLKeypoints);

    //if we want to.. pulish the keypoints
	if (d_publish)
	{
		PCLPointCloudPtr publishKeypoints(new PCLPointCloud);
		publishKeypoints->header.frame_id = "keypoints";
		publishKeypoints = publishize(clusterPCLs, keypoints);
		d_pubSampled.publish(publishKeypoints);
	}

	//compute the normals for each cluster
    vector<PointCloud<Normal>::Ptr> normals;
    normals = computeNormals(clusterPCLs);

	//get SHOT descriptors
    estimateDescriptors(clusterPCLs, keypoints, normals);
}

//overloaded function that extracts the required features for a single cluster 
PointCloud<descriptorType>::Ptr ObjectRecognizer::extractFeatures(PCLPointCloudPtr cluster)
{
	//Find the keypoints for the cluster
	vector<int> keypoints;
    PCLPointCloudPtr inputPtr(cluster);
    PointCloudOut sampled;
    sample(inputPtr, sampled);
    keypoints.insert(keypoints.begin(), sampled.points.begin(), sampled.points.end());

	//compute the normals
    PointCloud<Normal>::Ptr normal(new PointCloud<Normal>);
    normal = computeNormal(inputPtr);

	//compute the SHOT descriptors
    PointCloud<descriptorType>::Ptr SHOTOneClust(new PointCloud<descriptorType>);
    getSHOTDescriptors(inputPtr, keypoints, normal, *SHOTOneClust);

    return SHOTOneClust;
}
