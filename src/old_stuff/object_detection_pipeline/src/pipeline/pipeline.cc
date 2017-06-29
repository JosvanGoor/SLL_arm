#include <object_detection_pipeline/pipeline/pipeline.h>

Pipeline::Pipeline()
{
    loadFiles(); // first load the dataset
    start();    
    ros::spin();
}

void Pipeline::start()
{
    d_cloud_pub = d_node_handle.advertise<PCLPointCloud>("clusters", 1);
    d_pc_sub = d_node_handle.subscribe<PCLPointCloud>("camera/depth_registered/points", 1, &Pipeline::pcCallback, this);
}

void Pipeline::stop()
{
    d_pc_sub.shutdown();
}

void Pipeline::pcCallback(const PCLPointCloud::ConstPtr &cloudMsg)
{
    bool debug = true; // just for showing some debug messages

    clock_t tStart = clock();
    PCLPointCloudPtr pointcloud(new PCLPointCloud());


    clock_t tStartTiming = clock();
    // clean up the point cloud before using it. 
    vector<int> indices; // required by function but never used.
    removeNaNFromPointCloud(*cloudMsg, *pointcloud, indices);

    // segment out the plane
    d_segmentation.segmentPlane(pointcloud);

    if (debug)
        printf("plane segmentation: %.2fs\n", (double)(clock() - tStartTiming)/CLOCKS_PER_SEC);
    tStartTiming = clock();

    // cluster all the segments that are left 
    vector<PointIndices> cluster_indices = d_segmentation.cluster(pointcloud);

    ROS_INFO_STREAM("Nr clusters:" << cluster_indices.size());

    if (debug)
        printf("clustering: %.2fs\n", (double)(clock() - tStartTiming)/CLOCKS_PER_SEC);
    tStartTiming = clock();

    PCLPointCloudPtr tempCloud(new PCLPointCloud()); // temp cloud for publishing
   
    vector<ClusterObject> vClusterObjects(cluster_indices.size());
    
    for (size_t idx = 0; idx < cluster_indices.size(); ++idx) // gather all the clusters
    {
        PCLPointCloudPtr objectCluster (new PCLPointCloud());
        ClusterObject clusterObject;

        PointIndices point_indices = cluster_indices.at(idx);

        foreach (int index, point_indices.indices)          
        {
            Point p = pointcloud->points[index];             
            objectCluster->points.push_back(p);
        }   

        clusterObject.cloud = objectCluster;
        vClusterObjects.at(idx) = clusterObject;
    }

    if (debug)
        printf("cluster gathering %.2fs\n", (double)(clock() - tStartTiming)/CLOCKS_PER_SEC);
    tStartTiming = clock();

    for (size_t idx = 0; idx < vClusterObjects.size(); ++idx) // compute keypoints for all clusters
    {
      //  PCLPointCloudPtr keypoints(new PCLPointCloud());
        Keypoint<Point, int>::PointCloudOut keypoints;
        keypoints = d_keypointExtraction.compute(vClusterObjects.at(idx).cloud);
        vClusterObjects.at(idx).keypoints = keypoints;
    }

    if (debug)
        printf("keypoint extraction: %.2fs\n", (double)(clock() - tStartTiming)/CLOCKS_PER_SEC);
    tStartTiming = clock();

    for (size_t idx = 0; idx < vClusterObjects.size(); ++idx) // compute the descriptors for all clusters
    {
        PointCloud<DescriptorType>::Ptr descriptors(new PointCloud<DescriptorType>());
        descriptors = d_featureExtraction.compute(vClusterObjects.at(idx).keypoints, vClusterObjects.at(idx).cloud);
        vClusterObjects.at(idx).descriptors = descriptors;
    }

    if (debug)
        printf("descriptor computing: %.2fs\n", (double)(clock() - tStartTiming)/CLOCKS_PER_SEC);
    tStartTiming = clock();


    for (size_t idx = 0; idx < vClusterObjects.size(); ++idx) // try to match descriptors to database
    {
        d_matching.match(vClusterObjects.at(idx).descriptors, vClusterObjects.at(idx).keypoints, vClusterObjects.at(idx).cloud);
    }

    if (debug)
        printf("matching: %.2fs\n", (double)(clock() - tStartTiming)/CLOCKS_PER_SEC);
    tStartTiming = clock();


    // for publishing the cloud only
    for (size_t idc = 0; idc < vClusterObjects.size(); ++idc)
        *tempCloud += *vClusterObjects.at(idc).cloud;

    tempCloud->header.frame_id = "camera_link";
    d_cloud_pub.publish(tempCloud);

    printf("Total time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);

}


void Pipeline::loadFiles()
{
    clock_t tStart = clock();
    ROS_INFO_STREAM("LOADING FILES");

    vector<string> filenames;
 //   filenames.push_back("mountainDew");
    filenames.push_back("file");


    // load the objects .pcd files in memory
    for (size_t objectNr = 0; objectNr < filenames.size(); ++objectNr)
    {
        Object object;
        string name = filenames.at(objectNr);   
        object.name = name;

        for (size_t segmentNr = 0; segmentNr < 18; ++segmentNr)
        {
            PCLPointCloudPtr segment(new PCLPointCloud());
            stringstream ss;
            ss << "/home/rik/objects/" << filenames.at(objectNr)  << segmentNr << ".pcd";
            io::loadPCDFile(ss.str(), *segment);
            object.cloud = segment;

            // compute keypoints
            Keypoint<Point, int>::PointCloudOut objectKeypoints;
            objectKeypoints = d_keypointExtraction.compute(segment);

            object.keypoints = objectKeypoints;

            // compute descriptors
            PointCloud<DescriptorType>::Ptr model_descriptors(new PointCloud<DescriptorType>());
            model_descriptors = d_featureExtraction.compute(objectKeypoints, segment);

            object.descriptors = model_descriptors;
            d_objects.push_back(object);
        }
        
    }

    d_matching.loadFiles(d_objects);
    ROS_INFO_STREAM("DONE LOADING FILES");
    printf("Total time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);

}
