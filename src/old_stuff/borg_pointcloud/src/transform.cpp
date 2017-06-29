#include <ros/ros.h>
#include <tf/transform_listener.h>

// PCL includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PCLPointCloud;
typedef pcl::PointCloud<Point>::Ptr PCLPointCloudPtr;

// For publishing transformed PointCloud data. We don't want to create a new publisher every time
ros::Publisher transformedPointCloud_pub;

// If you declare a listener in the callback function, then it loses the history of the tf data.
// It therefore starts nagging about "Can't find frame_id 'base_link'" and stuff. So a global listener is required.
// The listener can't be initialized here, as ros::init has to be called first.
tf::TransformListener* listener;

// Global parameter values (I know, don't use global variables, but screw you)
double x_min;
bool remove_noise;

// This function is called whenever new point cloud data arrives
void depthCallback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{    
    // =======================================================
    // PointCloud frame_id transform: camera_link => base_link
    // =======================================================
    
    // Wait for transform to become available
    if (!listener->canTransform("/base_link", cloudMsg->header.frame_id, cloudMsg->header.stamp))
    {
        return;
    }

    // Transform PointCloud
    // After transform: X = distance from robot (0 = center of robot), Y = horizontal distance (0 = center of robot), Z = height (0 = ground)
    sensor_msgs::PointCloud2 cloudMsg_transformed;
    pcl_ros::transformPointCloud("/base_link", *cloudMsg, cloudMsg_transformed, *listener);
    
    // Convert PointCloud2 to PCL::PointCloud
    PCLPointCloudPtr pclCloud (new PCLPointCloud);
    pcl::fromROSMsg(cloudMsg_transformed, *pclCloud);
    
    // Discard Nao body and change leaf size using a voxelgrid filter
    PCLPointCloudPtr pclCloud_filtered_x (new PCLPointCloud);
    pcl::VoxelGrid<Point> voxelgrid_filter;
    voxelgrid_filter.setInputCloud(pclCloud);
    voxelgrid_filter.setLeafSize(0.01f, 0.01f, 0.01f);  // Leaf size: 1 cm
    voxelgrid_filter.setFilterFieldName("x");
    voxelgrid_filter.setFilterLimits(x_min, 5);
    voxelgrid_filter.filter(*pclCloud_filtered_x);
    
    // Remove noise using radius outlier removal
    if (remove_noise)
    {
        PCLPointCloudPtr pclCloud_cleaned (new PCLPointCloud);
        pcl::RadiusOutlierRemoval<Point> radius_outlier_removal;
        radius_outlier_removal.setInputCloud(pclCloud_filtered_x);
        radius_outlier_removal.setRadiusSearch(0.05);
        radius_outlier_removal.setMinNeighborsInRadius(10);
        radius_outlier_removal.filter(*pclCloud_cleaned);
        
        // Publish transformed and filtered PointCloud
        transformedPointCloud_pub.publish(*pclCloud_cleaned);
        return;
    }

    // Publish transformed PointCloud
    transformedPointCloud_pub.publish(*pclCloud_filtered_x);
}

int main(int argc, char** argv) 
{
    // ROS initialization stuff
	ros::init(argc, argv, "pointcloud_tf");
	ros::NodeHandle n;
	ros::Rate r(20);
	
	// Create listener and set global pointer
	tf::TransformListener temp_listener;
	listener = &temp_listener;

    // Parameters
	ros::param::param<double>("~x_min", x_min, 0.27);
    ros::param::param<bool>("~remove_noise", remove_noise, false);
    std::string input_topic;    ros::param::param<std::string>("~input_topic", input_topic, "camera/depth/points");
    std::string output_topic;   ros::param::param<std::string>("~output_topic", output_topic, "voxel_grid/output");
    	
    //Subscribe to kinect point cloud data
	ros::Subscriber depth_sub = n.subscribe<sensor_msgs::PointCloud2>(input_topic, 1, depthCallback);
	
	// Create a ROS publisher for the output PointClouds
    transformedPointCloud_pub = n.advertise<PCLPointCloud>(output_topic, 1);
    
    ros::spin();
    
    return 0;
}	
