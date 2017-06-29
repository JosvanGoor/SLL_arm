#include "objectRecognizer.ih"

/*
Contains:
processPCL(sensor_msgs): converts sensormsg to suitable pcl format
processPCL(PCLPointCloudPtr): preprocessing of the image and passing 
it on to feature extraction

Given an input image, clusters and cleans up the image. If d_publish
is true, publishes the clustered and the cleaned up image. Stores the 
dimensions of all found clusters and passes the clusters on to 
feature extraction 
*/

void ObjectRecognizer::processPCL(const sensor_msgs::PointCloud2ConstPtr &inputMsg)
{
	PCLPointCloudPtr PCLimg(new PCLPointCloud); 
	fromROSMsg(*inputMsg, *PCLimg); // convertion from ROS to PCL formats

	processPCL(PCLimg);
}

void ObjectRecognizer::processPCL(PCLPointCloudPtr PCLimg)
{
	// Remove planar surface and crop the width of the input image
	PCLPointCloudPtr surflessCloud(new PCLPointCloud); 

    double width = 0.6;
    double depth = 1.3;

    surflessCloud->header.frame_id = "(hopefully) planarless input";
 //   surflessCloud = filterView(PCLimg, width, depth);
//	surflessCloud = d_planRemover.removePlanarSurface(surflessCloud);

	// Set clustering variables
    d_objClusterer.minSize(100);
    d_objClusterer.maxSize(50000);
    d_objClusterer.tolerance(0.05);

    // Cluster objects
    vector<PointIndices> clusterIndices;  
    clusterIndices = d_objClusterer.cluster(PCLimg);
    
    // If we didn't find any cluster: terminate
    if (!clusterIndices.size())
        return;

    // Color all clusters with different grayscales
//	colorClusters(clusterIndices, surflessCloud);

	// Publish point cloud of clusters in the surfessless image
 //   if(d_publish)
 //       d_pubAll.publish(surflessCloud);
    
    // Clean up and remove everything from the point cloud except the clusters
    cleanUpImage(PCLimg, clusterIndices);
    
	clusterIndices = d_objClusterer.cluster(PCLimg);

	// Publish the clean point cloud of clusters
//    if(d_publish)
//        d_pubClean.publish(surflessCloud);
    
    //Get vector of all individual cluster PCLs
    vector<PCLPointCloudPtr> clusterPCLs;
    clusterPCLs = splitClusters(PCLimg, clusterIndices);

    //Calculate the dimensions of each cluster in view and store these
    getSizes(clusterPCLs);

    //Find the keypoints and calculate descriptors for each cluster
    extractFeatures(clusterPCLs);
}
