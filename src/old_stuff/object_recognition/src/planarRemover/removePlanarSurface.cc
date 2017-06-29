#include "planarRemover.ih"

PCLPointCloudPtr PlanarRemover::removePlanarSurface(PCLPointCloudPtr cloud)
{
	vector<int> ind;
	removeNaNFromPointCloud(*cloud, *cloud, ind);
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
//	seg.setMethodType(pcl::SAC_PROSAC);  // this method seems a little faster then the SAC_RANSAC method
	seg.setDistanceThreshold (0.015);  // 0.015 

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);		
	
	
//	segmentedCloud->height = cloud->height;
//	segmentedCloud->width = cloud->width;	 

	ExtractIndices<Point> extract;

	extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true); // true means remove planes
    extract.filter(*segmentedCloud);
 //	extract.filterDirectly(cloud);      
      
    // make sure the new cloud has no NaN points. 
    vector<int> indices; // required by function but never used.
    removeNaNFromPointCloud(*segmentedCloud, *segmentedCloud, indices);
 
  //  removeNaNFromPointCloud(*cloud, *segmentedCloud, indices);

 //   segmentedCloud = customRemovePlanar(segmentedCloud);

 //   segmentedCloud = customRemovePlanarGPU(segmentedCloud);
        
   
    return segmentedCloud;
}