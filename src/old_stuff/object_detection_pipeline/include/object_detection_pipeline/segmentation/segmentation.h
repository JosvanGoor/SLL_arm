#ifndef _H_SEGMENTATION
#define _H_SEGMENTATION

#include <object_detection_pipeline/utils/utils.h>

class Segmentation
{
	public:
		Segmentation();
        void segmentPlane(PCLPointCloudPtr &cloud);
        vector<PointIndices> cluster(PCLPointCloudPtr &cloud);
};

#endif