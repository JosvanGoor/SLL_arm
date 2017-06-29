#ifndef H_UTILS
#define H_UTILS

#include <iostream>
#include <vector> 
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>

#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/shot_omp.h>

#include <pcl/recognition/ransac_based/obj_rec_ransac.h>
#include <pcl/recognition/cg/geometric_consistency.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>

#include <pcl_ros/segmentation/extract_clusters.h>
#include <pcl/registration/ndt.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>


// boost includes
#include <boost/algorithm/string/predicate.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>


using namespace std;
using namespace sensor_msgs;
using namespace cv;
using namespace pcl;

typedef PointXYZRGB PCLPoint;
typedef pcl::PointCloud<PCLPoint> PCLPointCloud;
typedef pcl::PointCloud<PCLPoint>::Ptr PCLPointCloudPtr;

using boost::property_tree::ptree;

#ifdef __CDT_PARSER__
    #define foreach(a, b) for(a : b)
#else
    #define foreach(a, b) BOOST_FOREACH(a, b)
#endif


#endif