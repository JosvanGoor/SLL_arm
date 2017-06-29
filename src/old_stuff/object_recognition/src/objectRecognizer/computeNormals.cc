#include "objectRecognizer.ih"

/*
 * Contains:
 * computeNormals: compute set of normals for a set of clouds
 * computeNormal: compute the normal for a given cloud
 */

// compute set of normals for a set of clouds
vector<PointCloud<Normal>::Ptr> ObjectRecognizer::computeNormals(vector<PCLPointCloudPtr> inputClouds)
{
	vector<PointCloud<Normal>::Ptr> normals; //initialize the set of normals
	
	//For all clouds..
	for (size_t idx = 0; idx != inputClouds.size(); ++idx)
	{
		//compute the normal...
		PointCloud<Normal>::Ptr normal(new PointCloud<Normal>);
		normal = computeNormal(inputClouds[idx]);
		//and store it
		normals.push_back(normal);
	}

	return normals;
}

//compute the normal for a given cloud
PointCloud<Normal>::Ptr ObjectRecognizer::computeNormal(PCLPointCloudPtr inputCloud)
{
	// Create the normal estimation class, and pass the input dataset to it
	NormalEstimation<Point, Normal> ne;
	ne.setInputCloud (inputCloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	search::KdTree<Point>::Ptr tree (new search::KdTree<Point> ());
	ne.setSearchMethod (tree);

	// Output datasets
	PointCloud<Normal>::Ptr cloudNormals (new PointCloud<Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.03);

	// Compute the features
	ne.compute (*cloudNormals);
	
	return cloudNormals;
}