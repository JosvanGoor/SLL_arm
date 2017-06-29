#include "objectRecognizer.ih"

/*
Contains:
getKeypoints: Returns indices and pointclouds of downsampled clusters
sample: Downsamples an inputcloud

Given a vector of pointclouds, each corresponding to one object cluster,
use uniform sampling to get the keypoints for each objectcluster. Indices 
are stored as a vector of ints (called 'keypoints'), the downsampled 
pointclouds are stored in the vector 'outputs'. 
*/

vector<vector<int> > ObjectRecognizer::getKeypoints(vector<PCLPointCloudPtr> clusterPCLs, vector<PointCloudOut> outputs)
{
	//Loop over each pointcloud in the inputvector 'clusterPCLs' and find
	//the keypoints for each cluster ('keypoints' is the indices, 'outputs'
	//is the resulting pointclouds)
	vector<vector<int> > keypoints(clusterPCLs.size());

	for (size_t idx = 0; idx != clusterPCLs.size(); ++idx)
	{
		PointCloudOut sampled;
		sample(clusterPCLs[idx], sampled);
		outputs.push_back(sampled);
	    keypoints[idx].insert(keypoints[idx].begin(), sampled.points.begin(), sampled.points.end());
	}

	return keypoints;
}

void ObjectRecognizer::sample(PCLPointCloudPtr inputImg, PointCloudOut &output)
{
	//Given an input pointcloud, use uniform sampling to find a set of 
	//keypoints and store those in the poincloud 'output'.
	//
	//setRadiusSearch determines the magnitude of downsampling
	
	search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
	tree->setInputCloud(inputImg);

	UniformSampling<Point> uSampler;
	
	uSampler.setInputCloud(inputImg);
    uSampler.setRadiusSearch(0.015f);
	uSampler.setSearchMethod(tree);

	uSampler.compute(output);
}