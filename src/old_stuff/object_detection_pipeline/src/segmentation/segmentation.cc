#include <object_detection_pipeline/segmentation/segmentation.h>

Segmentation::Segmentation()
{
	
}

void Segmentation::segmentPlane(PCLPointCloudPtr &cloud)
{
    // TEMP FOR REDUCING FOV
    PassThrough<Point> passthrough_filter;
    passthrough_filter.setInputCloud(cloud);
    passthrough_filter.setFilterFieldName("z");
    passthrough_filter.setFilterLimits(0.4, 1.5f);
    passthrough_filter.filter(*cloud);  

    // filter Y     
    passthrough_filter.setInputCloud(cloud);
    passthrough_filter.setFilterFieldName("y");
  passthrough_filter.setFilterLimits(-0.2 , 0.2);
    passthrough_filter.filter(*cloud);

    // filter X 
    passthrough_filter.setInputCloud(cloud);
    passthrough_filter.setFilterFieldName("x");
    passthrough_filter.setFilterLimits(-0.20, 0.20);
  //  passthrough_filter.setFilterLimits(transformMatrix(0,3) - 0.20, transformMatrix(0,3) + 0.20);
    passthrough_filter.filter(*cloud);
    // END TEMP


    ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    SACSegmentation<Point> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);  // 1cm 

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);       
    
    ExtractIndices<Point> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    extract.setNegative(true); // true means remove planes
    extract.filter(*cloud); 

    vector<int> indices; // required by function but never used.
    removeNaNFromPointCloud(*cloud, *cloud, indices);
}

vector<PointIndices> Segmentation::cluster(PCLPointCloudPtr &cloud)
{

    vector<PointIndices> cluster_indices;
    PointCloud<PointXYZ>::Ptr xyzCloud (new PointCloud<PointXYZ>());
    xyzCloud->points.resize(cloud->size());

    #pragma omp parallel for
    for (size_t idx = 0; idx < cloud->size(); ++idx)
    {
        PointXYZ p;
        p.x = cloud->points[idx].x;
        p.y = cloud->points[idx].y;
        p.z = cloud->points[idx].z;
        xyzCloud->points.at(idx) = p;
    }

    // Creating the kdtree object for the search method of the extraction
    pcl::search::KdTree<PointXYZ>::Ptr tree (new pcl::search::KdTree<PointXYZ>());
    tree->setInputCloud(xyzCloud);

    EuclideanClusterExtraction<PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (1000);
    ec.setMaxClusterSize (80000);
    ec.setSearchMethod (tree);
    ec.setInputCloud(xyzCloud);
    ec.extract (cluster_indices);

    return cluster_indices;
}

