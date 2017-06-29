#ifndef _H_KEYPOINT
#define _H_KEYPOINT

#include <object_detection_pipeline/utils/utils.h>

class KeypointExtraction
{
    float d_uni_sample_size;
    
    public:
        KeypointExtraction();  
        Keypoint<Point, int>::PointCloudOut compute(PCLPointCloudPtr &cloud);
};

#endif