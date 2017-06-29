#include "objectDetection.h"

ObjectDetection::ObjectDetection()
{	
	cloud_pub = nh.advertise<PCLPointCloud>("pclTemp/output", 1);	
	
//	objectCloud = (new PCLPointCloud);
	objectCloud.header.frame_id = "camera_link";

	client_writer = nh.serviceClient<borg_pioneer::MemorySrv>("memory");
	client_reader = nh.serviceClient<borg_pioneer::MemoryReadSrv>("memory_read");
	
	d_currentSampleNr = 0;
	init();
}