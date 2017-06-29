#include <ros/ros.h>
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

#include <pcl/recognition/ransac_based/obj_rec_ransac.h>
#include <pcl/recognition/cg/geometric_consistency.h>


using namespace std;
using namespace ros;
using namespace pcl;

typedef PointXYZRGB Point;
typedef PointCloud<Point> PCLPointCloud;
typedef PointCloud<Point>::Ptr PCLPointCloudPtr;
typedef pcl::SHOT352 DescriptorType;

ros::Publisher cloud_pub;
ros::Publisher normal_pub;

pcl::PointCloud<pcl::Normal>::Ptr prev_normal;
bool first;
recognition::ObjRecRANSAC objRec(0.03f, 0.02f);

float uni_sample_size = 0.015f;
float descr_rad_ = 0.06f;
float normal_size = 0.03f;
float sd_distance = 0.25f; // the squared descriptor distance  (SHOT descriptor distances are between 0 and 1 by design)
float cg_size_ = 0.01f;
float cg_thresh_  = 5.0f;

pcl::PointCloud<DescriptorType>::Ptr stapler_descriptors (new pcl::PointCloud<DescriptorType> ());
pcl::PointCloud<DescriptorType>::Ptr pringles_descriptors (new pcl::PointCloud<DescriptorType> ());

PCLPointCloudPtr stapler_keypoints (new PCLPointCloud);
PCLPointCloudPtr pringles_keypoints (new PCLPointCloud);

PCLPointCloudPtr stapler_object(new PCLPointCloud);
PCLPointCloudPtr pringles_object(new PCLPointCloud);

void makeObjects()
{
    PCDReader reader;

    PointCloud<int> sampled_indices;
    UniformSampling<Point> detector;
    detector.setRadiusSearch(uni_sample_size);

    // compute normals
    pcl::NormalEstimationOMP<Point, pcl::Normal> ne;

    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point> ());
    ne.setSearchMethod (tree);
    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (normal_size);

    pcl::SHOTEstimationOMP<Point, pcl::Normal, DescriptorType> descr_est;
    descr_est.setRadiusSearch (descr_rad_);

    // first object
    
    reader.read("/home/rik/sudo/ros/catkin_ws/src/object_detection_pipeline_test/stapler.pcd", *stapler_object);

    // transform object to (0,0,0), take the middle point? 
    Eigen::Vector4f centroid;

    compute3DCentroid(*stapler_object, centroid);
    float x = centroid(0, 0);
    float y = centroid(1, 0);
    float z = centroid(2, 0);

    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    transform_1 (0,3) = -x;  // x
    transform_1 (1,3) = -y;  // y
    transform_1 (2,3) = -z;  // z

    pcl::transformPointCloud (*stapler_object, *stapler_object, transform_1);


    stapler_object->header.frame_id = "camera_rgb_optical_frame";
    detector.setInputCloud(stapler_object);    
    detector.compute(sampled_indices);     
    pcl::copyPointCloud (*stapler_object, sampled_indices.points, *stapler_keypoints);    
    ne.setInputCloud (stapler_object);
    ne.compute (*cloud_normals);    
    descr_est.setInputCloud (stapler_keypoints);
    descr_est.setInputNormals (cloud_normals);
    descr_est.setSearchSurface (stapler_object);
    descr_est.compute (*stapler_descriptors);

    stapler_keypoints->header.frame_id = "camera_rgb_optical_frame";


    // next object
    
    reader.read("/home/rik/sudo/ros/catkin_ws/src/object_detection_pipeline_test/pringles.pcd", *pringles_object);
    // transform object to (0,0,0), take the middle point? 
    compute3DCentroid(*pringles_object, centroid);
    x = centroid(0, 0);
    y = centroid(1, 0);
    z = centroid(2, 0);

    transform_1 = Eigen::Matrix4f::Identity();
    transform_1 (0,3) = -x;  // x
    transform_1 (1,3) = -y;  // y
    transform_1 (2,3) = -z;  // z

    pcl::transformPointCloud (*pringles_object, *pringles_object, transform_1);

    pringles_object->header.frame_id = "camera_rgb_optical_frame";
    detector.setInputCloud(pringles_object);    
    detector.compute(sampled_indices);
    pcl::copyPointCloud (*pringles_object, sampled_indices.points, *pringles_keypoints);
    ne.setInputCloud (pringles_object);
    ne.compute (*cloud_normals);
    descr_est.setInputCloud (pringles_keypoints);
    descr_est.setInputNormals (cloud_normals);
    descr_est.setSearchSurface (pringles_object);
    descr_est.compute (*pringles_descriptors);

    pringles_keypoints->header.frame_id = "camera_rgb_optical_frame";


}

void pcCallBack(const PCLPointCloud::ConstPtr &cloudMsg)
{
    PCLPointCloudPtr cloud_filtered (new PCLPointCloud);
    pcl::PassThrough<Point> pass;
    pass.setInputCloud (cloudMsg);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.10);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);
    
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-0.20, 0.20);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);
    
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-0.10, 0.20);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);

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

    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);       
    
    ExtractIndices<Point> extract;

    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true); // true means remove planes
    extract.filter(*cloud_filtered);

    if (cloud_filtered->size() > 0)
    {  
        vector<int> ind;
        removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, ind);

         if (cloud_filtered->size() <= 0) 
            return;

        Eigen::Vector4f centroid;
        Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
        compute3DCentroid(*cloud_filtered, centroid);
        float x = centroid(0, 0);
        float y = centroid(1, 0);
        float z = centroid(2, 0);

        transform_1 = Eigen::Matrix4f::Identity();
        transform_1 (0,3) = -x;  // x
        transform_1 (1,3) = -y;  // y
        transform_1 (2,3) = -z;  // z

        pcl::transformPointCloud (*cloud_filtered, *cloud_filtered, transform_1);


        PointCloud<int> sampled_indices;
        UniformSampling<Point> detector;
        detector.setRadiusSearch(uni_sample_size);
        detector.setInputCloud(cloud_filtered);    
        detector.compute(sampled_indices);
        
        PCLPointCloudPtr cloud_keypoints (new PCLPointCloud);
        pcl::copyPointCloud (*cloud_filtered, sampled_indices.points, *cloud_keypoints);
        
    //    ROS_INFO_STREAM("INPUT SIZE: " << cloud_filtered->size() << ", KEYPOINT SIZE: " << cloud_keypoints->size());

      //  PCDWriter writer;
      //  writer.write("/home/rik/sudo/ros/catkin_ws/src/object_detection_pipeline_test/pringles.pcd", *cloud_filtered);

        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimationOMP<Point, pcl::Normal> ne;
        ne.setInputCloud (cloud_filtered);
       //  ne.setInputCloud (cloud_filtered);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point> ());
        ne.setSearchMethod (tree);
        // Output datasets
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        cloud_normals->header.frame_id = "camera_rgb_optical_frame";
        // Use all neighbors in a sphere of radius 3cm
        ne.setRadiusSearch (normal_size);
        // Compute the features
        ne.compute (*cloud_normals);

        pcl::SHOTEstimationOMP<Point, pcl::Normal, DescriptorType> descr_est;

        pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());

        descr_est.setRadiusSearch (descr_rad_);
        descr_est.setInputCloud (cloud_keypoints);
        descr_est.setInputNormals (cloud_normals);
        descr_est.setSearchSurface (cloud_filtered);
        descr_est.compute (*model_descriptors);
        
        pcl::CorrespondencesPtr model_stapler_corrs (new pcl::Correspondences ());
        pcl::CorrespondencesPtr model_pringles_corrs (new pcl::Correspondences ());
        pcl::KdTreeFLANN<DescriptorType> match_search;
        match_search.setInputCloud (model_descriptors);

        //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
        for (size_t i = 0; i < stapler_descriptors->size (); ++i)
        {
            std::vector<int> neigh_indices (1);
            std::vector<float> neigh_sqr_dists (1);
            if (!pcl_isfinite (stapler_descriptors->at(i).descriptor[0])) //skipping NaNs
              continue;

            int found_neighs = match_search.nearestKSearch(stapler_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
            if(found_neighs == 1 && neigh_sqr_dists[0] < sd_distance) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
            {
              pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
              model_stapler_corrs->push_back(corr);
            }
        }

        //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
        for (size_t i = 0; i < pringles_descriptors->size (); ++i)
        {
            std::vector<int> neigh_indices (1);
            std::vector<float> neigh_sqr_dists (1);
            if (!pcl_isfinite (pringles_descriptors->at(i).descriptor[0])) //skipping NaNs
              continue;

            int found_neighs = match_search.nearestKSearch(pringles_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
            if(found_neighs == 1 && neigh_sqr_dists[0] < sd_distance) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
            {
              pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
              model_pringles_corrs->push_back (corr);
            }
        }

        std::cout << "Stapler Correspondences found: " << model_stapler_corrs->size () << std::endl;
        std::cout << "Pringles Correspondences found: " << model_pringles_corrs->size () << std::endl;


        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations_stapler;
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
        std::vector<pcl::Correspondences> clustered_corrs_stapler;
        std::vector<pcl::Correspondences> clustered_corrs;
       
        pcl::GeometricConsistencyGrouping<Point, Point> gc_clusterer;
        gc_clusterer.setGCSize (cg_size_);
        gc_clusterer.setGCThreshold (cg_thresh_);

        gc_clusterer.setInputCloud (cloud_keypoints);
        gc_clusterer.setSceneCloud (stapler_keypoints);
        gc_clusterer.setModelSceneCorrespondences (model_stapler_corrs);

        //gc_clusterer.cluster (clustered_corrs);
        gc_clusterer.recognize (rototranslations_stapler, clustered_corrs_stapler);

        std::cout << "Stapler instances found: " << rototranslations_stapler.size () << std::endl;

        gc_clusterer.setGCSize (cg_size_);
        gc_clusterer.setGCThreshold (cg_thresh_);
        gc_clusterer.setSceneCloud (pringles_keypoints);
        gc_clusterer.setModelSceneCorrespondences (model_pringles_corrs);

        //gc_clusterer.cluster (clustered_corrs);
        gc_clusterer.recognize (rototranslations, clustered_corrs);

        std::cout << "Pringles instances found: " << rototranslations.size () << std::endl;

        cout << endl;
        
        pcl::PointCloud<Point>::Ptr publish_models (new pcl::PointCloud<Point> ());

        *publish_models += *cloud_filtered;

        for (int i = 0; i < rototranslations.size (); ++i)
        {
            pcl::PointCloud<Point>::Ptr rotated_model (new pcl::PointCloud<Point> ());
            rotated_model->header.frame_id = cloud_filtered->header.frame_id;

            pcl::transformPointCloud (*pringles_object, *rotated_model, rototranslations[i]);        

            for (size_t idx = 0; idx < rotated_model->size(); ++idx)
            {
                rotated_model->points[idx].r = 255.0f;
                rotated_model->points[idx].g = 0.0f;
                rotated_model->points[idx].b = 0.0f;
            }

            *publish_models += *rotated_model;
        }

        for (int i = 0; i < rototranslations_stapler.size (); ++i)
        {
            pcl::PointCloud<Point>::Ptr rotated_model (new pcl::PointCloud<Point> ());
            rotated_model->header.frame_id = cloud_filtered->header.frame_id;
            pcl::transformPointCloud (*stapler_object, *rotated_model, rototranslations_stapler[i]);

            for (size_t idx = 0; idx < rotated_model->size(); ++idx)
            {
                rotated_model->points[idx].r = 255.0f;
                rotated_model->points[idx].g = 0.0f;
                rotated_model->points[idx].b = 0.0f;
            }

            *publish_models += *rotated_model;
        }
        
        publish_models->header.frame_id = cloud_filtered->header.frame_id;
        cloud_pub.publish(publish_models);
    }

}


int main(int argc, char** argv)
{
    makeObjects();
 //   PCDReader reader;
 //   PCLPointCloudPtr object(new PCLPointCloud);
 //   reader.read("/home/rik/sudo/ros/catkin_ws/src/object_detection_pipeline/test.pcd", *object);
 //   ROS_INFO_STREAM("SIZE: " << object->size());
    
 //   pcl::NormalEstimationOMP<Point, pcl::Normal> ne;
///   ne.setInputCloud (object);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point> ());
//    ne.setSearchMethod (tree);

    // Output datasets
 //   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
 //   ne.setRadiusSearch (0.03);

    // Compute the features
 //   ne.compute (*cloud_normals);
  
 //   objRec.addModel(*object, *cloud_normals, "sirop");
     /* 
    
    std::list<recognition::ObjRecRANSAC::Output> objOutput;
    objRec.recognize(*object, *cloud_normals, objOutput); 
    
    
    for (std::list<recognition::ObjRecRANSAC::Output>::iterator oneObjOutput = objOutput.begin(); oneObjOutput != objOutput.end() ; ++oneObjOutput)
        ROS_INFO_STREAM("                                                                                    OBJECT: " << oneObjOutput->object_name_ << ", " << oneObjOutput->match_confidence_);
    */
    first = true;
    pcl::PointCloud<pcl::Normal>::Ptr prev_normal (new pcl::PointCloud<pcl::Normal>);
    // ROS initialization stuff
	ros::init(argc, argv, "objectDetection");
	ros::NodeHandle nh;
	
	cloud_pub = nh.advertise<PCLPointCloud>("pclTemp/output", 1);
	normal_pub = nh.advertise<pcl::PointCloud<pcl::Normal> >("normals/output", 1);
	
	//Subscribe to point cloud data
	ros::Subscriber depth_sub = nh.subscribe<PCLPointCloud>("/camera/depth_registered/points", 1, pcCallBack);

	ros::spin();
}

