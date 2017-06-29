#include <object_detection_pipeline/keypointExtraction/keypointExtraction.h>

KeypointExtraction::KeypointExtraction()
{
    d_uni_sample_size = 0.005f; // std 0.015f
}

Keypoint<Point, int>::PointCloudOut KeypointExtraction::compute(PCLPointCloudPtr &cloud)
{

	search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
	tree->setInputCloud(cloud);

	UniformSampling<Point> detector;
	
	detector.setInputCloud(cloud);
    detector.setRadiusSearch(d_uni_sample_size);
	detector.setSearchMethod(tree);
	Keypoint<Point, int>::PointCloudOut keypoints;
	detector.compute(keypoints);
	return keypoints;
	/*
    PCLPointCloudPtr keypoints(new PCLPointCloud());
    PointCloud<int> sampled_indices;
    UniformSampling<Point> detector;
    detector.setRadiusSearch(d_uni_sample_size);
    detector.setInputCloud(cloud);    
    detector.compute(sampled_indices);

    pcl::copyPointCloud (*cloud, sampled_indices.points, *keypoints);
    return keypoints;

    */
}
