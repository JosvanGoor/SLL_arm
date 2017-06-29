#ifndef H_UTILS
#define H_UTILS

//ros
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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>

//boost
#include <boost/algorithm/string/predicate.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>


//namespaces
using namespace std;
using namespace pcl;
using namespace ros;

using boost::property_tree::ptree;

typedef PointXYZRGB Point;
typedef PointCloud<Point> PCLPointCloud;
typedef PointCloud<Point>::Ptr PCLPointCloudPtr;

#endif
