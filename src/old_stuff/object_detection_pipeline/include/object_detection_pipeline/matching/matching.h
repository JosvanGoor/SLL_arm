#ifndef _H_MATCHING
#define _H_MATCHING

#include <object_detection_pipeline/utils/utils.h>

class Matching
{
	vector<Object>  d_objects;

	public:
		Matching();
		void match(pcl::PointCloud<DescriptorType>::Ptr &descriptors, Keypoint<Point, int>::PointCloudOut &keypoints, PCLPointCloudPtr	&cloud);
		void loadFiles(vector<Object>  &objects);

};

#endif
