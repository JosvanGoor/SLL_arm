
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

// PointCloud includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/feature.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

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

ros::Publisher publisher;

void pcCallBack(const sensor_msgs::PointCloud2ConstPtr &cloudMsg)
{
	PCLPointCloudPtr receivedCloud(new PCLPointCloud);
	PCLPointCloud unmodCloud;



	fromROSMsg(*cloudMsg, *receivedCloud); // convertion from ROS to PCL formats
	fromROSMsg(*cloudMsg, unmodCloud); // convertion from ROS to PCL formats

	// Remove planar surface
	PCLPointCloudPtr surflessCloud(new PCLPointCloud);
	surflessCloud->header.frame_id = "camera_link";

	vector<int> ind;
	removeNaNFromPointCloud(*receivedCloud, *surflessCloud, ind);

	PCLPointCloudPtr segmentedCloud (new PCLPointCloud);

	ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	SACSegmentation<Point> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.015);  // 1.5cm

	seg.setInputCloud(surflessCloud);
	seg.segment(*inliers, *coefficients);

	ExtractIndices<Point> extract;

	extract.setInputCloud(surflessCloud);
	extract.setIndices(inliers);
	extract.setNegative(true); // true means remove planes
	extract.filter(*segmentedCloud);

	segmentedCloud->header.frame_id = "camera_link";
	publisher.publish(*segmentedCloud);



	// make sure the new cloud has no NaN points.
	vector<int> indices; // required by function but never used.
	removeNaNFromPointCloud(*segmentedCloud, *segmentedCloud, indices);

	if (segmentedCloud->points.size() < 1)
		return;

	std::vector<pcl::PointIndices> clusters;     // Cluster information is stored here

	pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);

	tree->setInputCloud(segmentedCloud);
	EuclideanClusterExtraction<Point> ec;

	ec.setClusterTolerance(0.02);       // in meters
	ec.setMinClusterSize(1000);         // Minimal points that must belong to a cluster
	ec.setMaxClusterSize(50000);      // Maximal points that must belong to a cluster
	ec.setSearchMethod(tree);
	ec.setInputCloud(segmentedCloud);
	ec.extract(clusters);

	if (clusters.size() == 0)
		return;

	ROS_INFO_STREAM("Objects found: " << clusters.size());
	int width = 480;
	int height = 640;
	cv::Mat newImage(width, height, CV_8UC3);

	int count = 0;

	for (size_t idx = 0; idx < width; ++idx)
	{
		for (size_t idy = 0; idy < height; ++idy)
		{
			int b =	unmodCloud.points[count].b;
			int g =	unmodCloud.points[count].g;
			int r =	unmodCloud.points[count].r;
			newImage.at<cv::Vec3b>(idx, idy)[0] = (unsigned char)b;
			newImage.at<cv::Vec3b>(idx, idy)[1] = (unsigned char)g;
			newImage.at<cv::Vec3b>(idx, idy)[2] = (unsigned char)r;
			++count;
		}
	}

	for (size_t idc = 0; idc < clusters.size(); ++idc)
	{
		PCLPointCloudPtr pclCloud_segmented (new PCLPointCloud);
		pclCloud_segmented->header.frame_id = "camera_link";
		pcl::PointIndices point_indices = clusters.at(idc);

		Point minPtX, minPtY, maxPtX, maxPtY;
		minPtX.x = 99.0f;
		minPtY.y = 99.0f;

		maxPtX.x = -99.0f;
		maxPtY.y = -99.0f;

		foreach (int index, point_indices.indices)
		{
			Point p = segmentedCloud->points[index];

			if (p.x < minPtX.x)
				minPtX = p;

			if (p.x > maxPtX.x)
				maxPtX = p;

			if (p.y < minPtY.y)
				minPtY = p;

			if (p.y > maxPtY.y)
				maxPtY = p;
			//pclCloud_segmented->points.push_back(p);
		}

		int left = 0;
		int right = 0;
		int top = 0;
		int bottom = 0;

		for (size_t idx = 0; idx < unmodCloud.points.size(); ++idx)
		{
			Point p = unmodCloud.points[idx];
			if (minPtX.x == p.x && minPtX.y == p.y && minPtX.z == p.z)
				left = idx;

			if (minPtY.x == p.x && minPtY.y == p.y && minPtY.z == p.z)
				bottom = idx;

			if (maxPtX.x == p.x && maxPtX.y == p.y && maxPtX.z == p.z)
				right = idx;

			if (maxPtX.x == p.x && maxPtX.y == p.y && maxPtX.z == p.z)
				top = idx;

			if (left != 0 && right != 0 &&
				top != 0 && bottom != 0)
				break;
		}

		size_t xLeft, xRight, yTop, yBottom;

		int div = unmodCloud.width;
		int bufferSize = 2;
		xLeft = left % div - bufferSize < 0 ? 0 : left % div - bufferSize ;
		xRight =  right % div + bufferSize > 640 ? 639 : right % div + bufferSize;

		yBottom = ceil(top / div) + bufferSize > 480 ? 479 : ceil(top / div) + bufferSize;
		yTop = ceil(bottom / div) - bufferSize < 0 ? 0 : ceil(bottom / div) - bufferSize;

		ROS_INFO_STREAM("xLeft: " << xLeft);
		cv::Rect roi(cv::Point(xLeft, yTop),
						cv::Point(xRight, yBottom));

		cv::imshow("object", newImage(roi));
		cv::waitKey(2000);
	}


}

int main(int argc, char **argv)
{
	// ROS initialization stuff
	ros::init(argc, argv, "object2d");
	ros::NodeHandle n;

	//Subscribe to point cloud data
	ros::Subscriber depth_sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, pcCallBack);
	publisher = n.advertise<PCLPointCloud>("simpleFeature/output", 1);

	ros::spin();

}
