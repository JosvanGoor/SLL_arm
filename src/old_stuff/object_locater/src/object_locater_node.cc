#include <iostream>
#include <ros/ros.h>

#include <tf/transform_listener.h>

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
#include <pcl/filters/statistical_outlier_removal.h>

// boost includes
#include <boost/algorithm/string/predicate.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>

#include <object_locater/objectLocation.h>
#include <object_locater/locateObjectAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<object_locater::locateObjectAction> Server;

using namespace std;
using namespace ros;
using namespace pcl;

using boost::property_tree::ptree;

typedef PointXYZRGB Point;
typedef PointCloud<Point> PCLPointCloud;
typedef PointCloud<Point>::Ptr PCLPointCloudPtr;

#ifdef __CDT_PARSER__
    #define foreach(a, b) for(a : b)
#else
    #define foreach(a, b) BOOST_FOREACH(a, b)
#endif


Publisher cloud_pub;
tf::TransformListener *tfTransformer;
Publisher publisher;

bool hasFoundObject;
bool findOpenSpace;
bool process;
int counter;
const int maxCount = 20;
object_locater::locateObjectResult objectLocation;

size_t clearBuffer;


void execute(const object_locater::locateObjectGoalConstPtr &goal, Server *as)
{
    ROS_INFO_STREAM("Started object locater");

    counter = 0;
    hasFoundObject = false; 

    // reset the message
    objectLocation.object.correct = false;
    objectLocation.object.x = 0.0f;
    objectLocation.object.y = 0.0f;
    objectLocation.object.z = 0.0f;
    objectLocation.object.thetaX = 0.0f;
    objectLocation.object.thetaY = 0.0f;
    objectLocation.object.thetaZ = 0.0f;

    // buffer
    clearBuffer = 0;

    process = true; // start processing point cloud 
    
    if (goal->findEmptySpace == true)
    	findOpenSpace = true;
    else
    	findOpenSpace = false;


    ros::Rate loop_rate(20); // 5Hz polling, we are not in a hurry 

    while (process == true)
        loop_rate.sleep();

    if (hasFoundObject == true)
    {
        as->setSucceeded(objectLocation);
        return;
    }
    
    if (hasFoundObject == false)
    {
        as->setAborted(objectLocation);
    }

}

float toRad(float deg)
{
    return deg * (M_PI / 180.0f);
}


void pcCallback(const sensor_msgs::PointCloud2ConstPtr &cloudMsg)
{
    if (process == true)
    {
    	++clearBuffer;

    	if (clearBuffer < 20) // just clear the point cloud, maybe its lagging behind
    		return;
   //     ROS_INFO_STREAM("POINT CLOUD");

        ++counter; // higher the counter for time-out check

        if (counter == maxCount)
        {
            process = false;
            return;
        }

        tfTransformer->waitForTransform("/mico_base_link", "/front_xtion_link", ros::Time::now(), ros::Duration(0.05));
    
        if (!tfTransformer->canTransform("/mico_base_link", cloudMsg->header.frame_id, cloudMsg->header.stamp))
        {
            ROS_INFO_STREAM("No transform found");
            return;
        }
    
        PCLPointCloudPtr pointcloud(new PCLPointCloud);
        sensor_msgs::PointCloud2 cloudMsg_transformed;
        pcl_ros::transformPointCloud("/mico_base_link", *cloudMsg, cloudMsg_transformed, *tfTransformer);
         
        fromROSMsg(cloudMsg_transformed, *pointcloud);
  
//        fromROSMsg(*cloudMsg, *pointcloud);
        PassThrough<Point> passthrough_filter;
        passthrough_filter.setInputCloud(pointcloud);
        passthrough_filter.setFilterFieldName("z");
        passthrough_filter.setFilterLimits(-0.05, 1.5);
        passthrough_filter.filter(*pointcloud);	

        // filter Y		
        passthrough_filter.setInputCloud(pointcloud);
        passthrough_filter.setFilterFieldName("y");
        passthrough_filter.setFilterLimits(-1.0, -0.35);
        passthrough_filter.filter(*pointcloud);
        
        // filter X 
        passthrough_filter.setInputCloud(pointcloud);
        passthrough_filter.setFilterFieldName("x");
        passthrough_filter.setFilterLimits(-0.40, 0.40);
        passthrough_filter.filter(*pointcloud);

        vector<int> ind;
        removeNaNFromPointCloud(*pointcloud, *pointcloud, ind);

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

        seg.setInputCloud(pointcloud);
        seg.segment(*inliers, *coefficients);		
        
        ExtractIndices<Point> extract;

        extract.setInputCloud(pointcloud);
        extract.setIndices(inliers);

        PCLPointCloudPtr planes(new PCLPointCloud);
        extract.setNegative(false); // extract the plane
        extract.filter(*planes);

        extract.setNegative(true); // true means remove planes
        extract.filter(*pointcloud);        

        vector<pcl::PointIndices> cluster; 
        pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
        EuclideanClusterExtraction<Point> ec;

        if (pointcloud->size() != 0)
        {
        	tree->setInputCloud(pointcloud);


			ec.setClusterTolerance(0.08);       // in meters
			ec.setMinClusterSize(100);       // Minimal points that must belong to a cluster
			ec.setMaxClusterSize(40000);      // Maximal points that must belong to a cluster
			ec.setSearchMethod(tree);
			ec.setInputCloud(pointcloud);
			ec.extract(cluster);
        }

        float avgX = 0.0f;
        float avgY = 0.0f;
        float z = 0.0f;
        
        cloud_pub.publish(pointcloud);

        if (findOpenSpace == true)
        {
        	Point objectPoint;
        	if (cluster.size() == 0) // clean area
        	{
        		objectPoint.x = 0.0f;
				objectPoint.y = -0.55f; // put the object 55cm in front of the arm
        	}
        	else // put object right off all other objects
        	{
        		float mostRight = 10.0f; // point 10m to the left

        		// check all the locations of all the objects
        		for (size_t idx = 0; idx < cluster.size(); ++idx)
				{
					PCLPointCloudPtr objectCloud(new PCLPointCloud);
					pcl::PointIndices point_indices = cluster.at(idx);

					foreach (int index, point_indices.indices)
					{
						Point p = pointcloud->points[index];
						objectCloud->points.push_back(p);
					}

					Point minPt, maxPt;
					getMinMax3D(*objectCloud, minPt, maxPt);

					if (minPt.x < mostRight) // object is left of mostRight value
					{
						mostRight = minPt.x;
					}
				}

        		objectPoint.x = mostRight - 0.15f; // move 15cm to the right of most right object
        		objectPoint.y = -0.5f;
        	}

			// we need the height of the table
			StatisticalOutlierRemoval<Point> sor;
			sor.setInputCloud(planes);
			sor.setMeanK(50);
			sor.setStddevMulThresh(1.0);
			sor.filter(*planes);

        	pcl::search::KdTree<Point>::Ptr treeplanes (new pcl::search::KdTree<Point>);
			treeplanes->setInputCloud(planes);
			ec.setSearchMethod(treeplanes);
			ec.setInputCloud(planes);
			ec.extract(cluster);

			float height = -10.0f;

			for (size_t idx = 0; idx < cluster.size(); ++idx)
			{
				PCLPointCloudPtr clusterCloud(new PCLPointCloud);
				pcl::PointIndices point_indices = cluster.at(idx);

				foreach (int index, point_indices.indices)
				{
					Point p = planes->points[index];
					clusterCloud->points.push_back(p);
				}

				Point minPt, maxPt;
				getMinMax3D(*clusterCloud, minPt, maxPt);

				if (fabs(maxPt.z - minPt.z) <= 0.05 and maxPt.z > height)
					height = maxPt.z;
			}

			ROS_INFO_STREAM("height table: " << height);
			objectLocation.object.correct = true;
			objectLocation.object.x = objectPoint.x;
			objectLocation.object.y = objectPoint.y; // include some distance from the object
			objectLocation.object.z = height;

			objectLocation.object.thetaX = toRad(90); // always 90 degree, so hot
			objectLocation.object.thetaY = 0.0f;
			objectLocation.object.thetaZ = 0.0f;

			hasFoundObject = true; // extra check to send feedback success
			process = false; // done
			return; // done here

        }

        if (cluster.size() == 0)
        {
            ROS_INFO_STREAM("No clusters!!");
            return;
        }

        ROS_INFO_STREAM(cluster.size() << " cluster(s) found");

        // there should only be one cluster 

        // testing stuff
        Point objectPoint;
        objectPoint.x = -99.0f; // a point to the right 
        objectPoint.z = -99.0f; // a very low point

        Point maxP, minP;

        PCLPointCloudPtr pubCloud(new PCLPointCloud);
        pubCloud->header.frame_id = "camera_rgb_frame";

        for (size_t idx = 0; idx < cluster.size(); ++idx)
        {

            PCLPointCloudPtr objectCloud(new PCLPointCloud);
            pcl::PointIndices point_indices = cluster.at(idx);

            foreach (int index, point_indices.indices)			
            {
                Point p = pointcloud->points[index];				
                objectCloud->points.push_back(p);
            }  			
            
            *pubCloud += *objectCloud;
            Point minPt, maxPt;
            getMinMax3D(*objectCloud, minPt, maxPt);

            // Get the avg x,y of cluster, and the max height of the object
            avgX = (minPt.x + maxPt.x) / 2;
            avgY = (minPt.y + maxPt.y) / 2;
            z = maxPt.z;

        //    if (z > objectPoint.z )
            if (avgX > objectPoint.x)
            {
                objectPoint.x = avgX;
                objectPoint.y = avgY;
                objectPoint.z = z;
                maxP.z = maxPt.z;
                minP.z = minPt.z;
                maxP.x = maxPt.x;
                minP.x = minPt.x;
            }

          //  ROS_INFO_STREAM("Point: " << avgX << ", " << avgY << ", " << z);
        }


        StatisticalOutlierRemoval<Point> sor;
        sor.setInputCloud(planes);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        sor.filter(*planes);
      //  cloud_pub.publish(planes);

        tree->setInputCloud(planes);
        ec.setSearchMethod(tree);
        ec.setInputCloud(planes);
        ec.extract(cluster);

        ROS_INFO_STREAM(cluster.size() << " planes detected");

        float height = 0.0f;

        for (size_t idx = 0; idx < cluster.size(); ++idx)
        {
            PCLPointCloudPtr clusterCloud(new PCLPointCloud);
            pcl::PointIndices point_indices = cluster.at(idx);

            foreach (int index, point_indices.indices)          
            {
                Point p = planes->points[index];                
                clusterCloud->points.push_back(p);
            }           
            
            Point minPt, maxPt;
            getMinMax3D(*clusterCloud, minPt, maxPt);

            if (maxPt.z > height) // take the highest plane
                height = maxPt.z;
        }

        ROS_INFO_STREAM("Point: " << objectPoint.x << ", " << objectPoint.y << ", " << objectPoint.z);
        ROS_INFO_STREAM("Table height: " << height);
        ROS_INFO_STREAM("Height: " <<  maxP.z - height);
        ROS_INFO_STREAM("LEFT: " << maxP.x << ", RIGHT: " << minP.x);
        
        hasFoundObject = true; // extra check that the object was found
        process = false; // stop the processing

        // set the feedback result 
        objectLocation.object.correct = true;
        objectLocation.object.x = objectPoint.x;
        objectLocation.object.y = objectPoint.y; // include some distane from the object
        objectLocation.object.z = objectPoint.z - 0.02f; // max height of object minus 2 cm?

        // always grasp from the front
        objectLocation.object.thetaX = toRad(90);
        objectLocation.object.thetaY = 0.0f;
        objectLocation.object.thetaZ = 0.0f;  
        objectLocation.object.height = maxP.z - height; 
        objectLocation.object.tableHeight = height; 
        objectLocation.object.xLeft = maxP.x;
        objectLocation.object.xRight = minP.x;
    }
}



int main(int argc, char **argv)
{
    hasFoundObject = false; 
    process = false; // point cloud does not need to be processed

	ros::init(argc, argv, "mico_testing");
	ros::NodeHandle node_handle(""); 

	ros::Subscriber depth_sub = node_handle.subscribe<sensor_msgs::PointCloud2>("front_xtion/depth_registered/points", 1, pcCallback);
	cloud_pub = node_handle.advertise<PCLPointCloud>("pclTemp/output", 1);
    
   // publisher = node_handle.advertise<object_locater::objectLocation>("objectDetection/location", 1);

    // Create listener and set global pointer
    tf::TransformListener temp_listener;
    tfTransformer = &temp_listener;

    Server server(node_handle, "locate_object", boost::bind(&execute, _1, &server), false);
    server.start();

	ros::spin();
	
/*
	ros::Rate loop_rate(10);
    while (ros::ok())
    { 
        ros::spinOnce();
        loop_rate.sleep();
    }

*/
}


