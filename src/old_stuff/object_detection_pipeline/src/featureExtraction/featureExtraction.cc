#include <object_detection_pipeline/featureExtraction/featureExtraction.h>

FeatureExtraction::FeatureExtraction()
{
    d_normal_size = 0.06f; // std 0.03f
    d_descr_rad = 0.07f; // std 0.06f
}


PointCloud<DescriptorType>::Ptr FeatureExtraction::compute(Keypoint<Point, int>::PointCloudOut &keypoints, PCLPointCloudPtr &cloud)
{
    pcl::NormalEstimationOMP<Point, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point> ());
    ne.setSearchMethod(tree);
    // Use all neighbors in a sphere of radius d_normal_size
    ne.setRadiusSearch(d_normal_size);
    
    // Output datasets
    pcl::PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>);    
    // Compute the features
    ne.compute (*cloud_normals);

    PointIndices::Ptr keyIndices(new PointIndices);
    vector<int> vkeypoints;
    vkeypoints.insert(vkeypoints.end(), keypoints.points.begin(), keypoints.points.end());
    keyIndices->indices = vkeypoints;

    pcl::SHOTColorEstimationOMP<Point, Normal, DescriptorType> descr_est;
 //   pcl::SHOTEstimationOMP<Point, Normal, DescriptorType> descr_est;

    pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new PointCloud<DescriptorType>());
/*
    descr_est.setSearchMethod(tree);
    descr_est.setInputCloud(cloud);
    descr_est.setInputNormals(cloud_normals);
    descr_est.setRadiusSearch(d_descr_rad);
    descr_est.compute(*model_descriptors);
 */
    descr_est.setSearchMethod(tree);
    descr_est.setRadiusSearch(d_descr_rad);
    descr_est.setInputCloud(cloud);
    descr_est.setInputNormals(cloud_normals);
    descr_est.setIndices (keyIndices); //keypoints
    descr_est.compute (*model_descriptors);

    return model_descriptors;
}
