#include <iostream>
#include <ros/ros.h>

#include <time.h>

// PointCloud includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>

#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/shot_omp.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// The GPU specific stuff here
#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.hpp>
#include <pcl/gpu/segmentation/gpu_extract_clusters.h>
#include <pcl/gpu/segmentation/impl/gpu_extract_clusters.hpp>


// boost includes
#include <boost/algorithm/string/predicate.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>


using namespace std;
using namespace ros;
using namespace pcl;

using boost::property_tree::ptree;

typedef PointXYZ Point;
typedef PointCloud<Point> PCLPointCloud;
typedef PointCloud<Point>::Ptr PCLPointCloudPtr;

#ifdef __CDT_PARSER__
    #define foreach(a, b) for(a : b)
#else
    #define foreach(a, b) BOOST_FOREACH(a, b)
#endif


ros::Publisher cloud_pub;

void pcCallback(const PCLPointCloud::ConstPtr &cloudMsg)
{
    clock_t tStart = clock();

    PCLPointCloudPtr pointcloud(new PCLPointCloud());

    ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    SACSegmentation<Point> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.02);  // 0.015 

    seg.setInputCloud(cloudMsg);
    seg.segment(*inliers, *coefficients);       
    
    ExtractIndices<Point> extract;

    extract.setInputCloud(cloudMsg);
    extract.setIndices(inliers);

    PCLPointCloudPtr planes(new PCLPointCloud);
    extract.setNegative(false); // extract the plane
    extract.filter(*planes);

    extract.setNegative(true); // true means remove planes
    extract.filter(*pointcloud); 

    vector<int> indices; // required by function but never used.
    removeNaNFromPointCloud(*pointcloud, *pointcloud, indices);
    int j = 0;
    printf("Plane segmentation cpu Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);

    tStart = clock();
//  /*
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (pointcloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (1000);
    ec.setMaxClusterSize (40000);
    ec.setSearchMethod (tree);
    ec.setInputCloud(pointcloud);
    ec.extract (cluster_indices);

    printf("CPU Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
    ROS_INFO_STREAM("Nr of clusters: " << cluster_indices.size() );

    PCLPointCloudPtr tempCloud(new PCLPointCloud());

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
        {
            cloud_cluster->points.push_back (pointcloud->points[*pit]); //*
            tempCloud->points.push_back(pointcloud->points[*pit]);
        }

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        j++;
    }
 //   */
   // /*

    tStart = clock();
    pcl::gpu::Octree::PointCloud cloud_device;
    cloud_device.upload(pointcloud->points);
  
    pcl::gpu::Octree::Ptr octree_device (new pcl::gpu::Octree);
    octree_device->setCloud(cloud_device);
    octree_device->build();


    std::vector<pcl::PointIndices> cluster_indices_gpu;
    pcl::gpu::EuclideanClusterExtraction gec;
    gec.setClusterTolerance (0.02); // 2cm
    gec.setMinClusterSize (1000);
    gec.setMaxClusterSize (40000);
    gec.setSearchMethod (octree_device);
    gec.setHostCloud(pointcloud);
    gec.extract (cluster_indices_gpu); 

    printf("GPU Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
    ROS_INFO_STREAM("Nr of clusters: " << cluster_indices_gpu.size() );

//    PCLPointCloudPtr tempCloud(new PCLPointCloud());

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_gpu.begin (); it != cluster_indices_gpu.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_gpu (new pcl::PointCloud<pcl::PointXYZ>);

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
        {
            cloud_cluster_gpu->points.push_back (pointcloud->points[*pit]); //*
            tempCloud->points.push_back(pointcloud->points[*pit]);
        }
        
        cloud_cluster_gpu->width = cloud_cluster_gpu->points.size ();
        cloud_cluster_gpu->height = 1;
        cloud_cluster_gpu->is_dense = true;

    //    std::cout << "PointCloud representing the Cluster: " << cloud_cluster_gpu->points.size () << " data points." << std::endl;
       
        j++;
    }

  // */

  tempCloud->header.frame_id = "camera_rgb_optical_frame";
//  cloud_pub.publish(tempCloud);


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mico_testing");
    ros::NodeHandle node_handle(""); 

    ros::Subscriber depth_sub = node_handle.subscribe<PCLPointCloud>("camera/depth_registered/points", 1, pcCallback);
    cloud_pub = node_handle.advertise<PCLPointCloud>("pclTemp/output", 1);
    
   // publisher = node_handle.advertise<object_locater::objectLocation>("objectDetection/location", 1);

    ros::spin();
}