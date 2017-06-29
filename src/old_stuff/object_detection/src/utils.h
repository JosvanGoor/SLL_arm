#ifndef H_UTILS
#define H_UTILS


#include <iostream>
#include <numeric>
#include <cstdlib>

// openCL
//#include <CL/opencl.h>

// ros includes
#include <ros/ros.h>
#include <ros/package.h>

// PointCloud includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/features/feature.h>
#include <pcl_ros/segmentation/sac_segmentation.h>
#include <pcl_ros/segmentation/extract_clusters.h>
#include <pcl_ros/filters/extract_indices.h>
#include <pcl/common/common.h>

// boost includes
#include <boost/algorithm/string/predicate.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

//opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/objdetect/objdetect.hpp>

// Borg Memory service
#include <borg_pioneer/MemoryReadSrv.h>
#include <borg_pioneer/MemorySrv.h>


// namespaces
using namespace std;
using namespace pcl;
using namespace ros;

using boost::property_tree::ptree;

typedef PointXYZRGB Point;
typedef PointCloud<Point> PCLPointCloud;
typedef PointCloud<Point>::Ptr PCLPointCloudPtr;

#ifdef __CDT_PARSER__
    #define foreach(a, b) for(a : b)
#else
    #define foreach(a, b) BOOST_FOREACH(a, b)
#endif


#endif

