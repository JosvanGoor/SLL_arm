#ifndef _H_PIPELINE
#define _H_PIPELINE

#include <object_detection_pipeline/utils/utils.h>
#include <object_detection_pipeline/receivePCL/receivePCL.h>
#include <object_detection_pipeline/segmentation/segmentation.h>
#include <object_detection_pipeline/keypointExtraction/keypointExtraction.h>
#include <object_detection_pipeline/featureExtraction/featureExtraction.h>
#include <object_detection_pipeline/matching/matching.h>


class Pipeline
{
    // submodules
    ReceivePCL d_receivePCL;
    Segmentation d_segmentation;
    KeypointExtraction d_keypointExtraction;
    FeatureExtraction d_featureExtraction;
    Matching d_matching;


    ros::NodeHandle d_node_handle; 

    ros::Subscriber d_pc_sub;
    ros::Publisher d_cloud_pub;

    vector<Object> d_objects;

    public:
        Pipeline();
        void start();
        void stop();

    private:
        void pcCallback(const PCLPointCloud::ConstPtr &cloudMsg);
        void loadFiles();

};

#endif