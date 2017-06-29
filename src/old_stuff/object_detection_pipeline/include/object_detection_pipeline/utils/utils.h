#ifndef _H_UTILS
#define _H_UTILS

#include <iostream>


#include <ros/ros.h>
#include <time.h>

// pcl stuff
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/filters/passthrough.h>

// boost includes
#include <boost/algorithm/string/predicate.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>

#include <boost/thread/thread.hpp>


using namespace std;
using namespace ros;
using namespace pcl;

using boost::property_tree::ptree;


typedef PointXYZRGB Point;
typedef PointCloud<Point> PCLPointCloud;
typedef PointCloud<Point>::Ptr PCLPointCloudPtr;

//Typedef for feature extraction
//typedef pcl::SHOT352 DescriptorType;
typedef pcl::SHOT1344 DescriptorType;


#ifdef __CDT_PARSER__
    #define foreach(a, b) for(a : b)
#else
    #define foreach(a, b) BOOST_FOREACH(a, b)
#endif


struct Object
{
	PCLPointCloudPtr cloud;
	PointCloud<DescriptorType>::Ptr descriptors;
	Keypoint<Point, int>::PointCloudOut keypoints;
	string name;
};

struct FoundObject
{
	string name;
	CorrespondencesPtr model_corrs;
	float avgDistance;
	PCLPointCloudPtr cloud;
};

struct ClusterObject
{
	string name;
	PCLPointCloudPtr cloud;
	PointCloud<DescriptorType>::Ptr descriptors;
	Keypoint<Point, int>::PointCloudOut keypoints;
};

struct CorrectObject
{
	string name;
	float distance;
};

#endif
