#ifndef _H_FEATURE
#define _H_FEATURE

#include <object_detection_pipeline/utils/utils.h>


class FeatureExtraction
{
    float d_normal_size;
    float d_descr_rad; 
    
    public:
        FeatureExtraction();
        PointCloud<DescriptorType>::Ptr compute(Keypoint<Point, int>::PointCloudOut &keypoints, PCLPointCloudPtr &cloud);
};

#endif