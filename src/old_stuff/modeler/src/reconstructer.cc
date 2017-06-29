#include <modeler/reconstructer.h>

Reconstructer::Reconstructer()
{
	d_cloud_pub = d_nh.advertise<PCLPointCloud>("reconstruction/model", 1);
}


void Reconstructer::reconstruct(vector<PCLPointCloud> vObject)
{
	PCLPointCloudPtr object(new PCLPointCloud());
	*object = vObject.at(0);
	

	for (size_t idx = 0; idx < vObject.size() - 1; ++idx)
	{
		PCLPointCloudPtr newCloud(new PCLPointCloud());
		PCLPointCloudPtr source(new PCLPointCloud());
		PCLPointCloudPtr target(new PCLPointCloud());

		*source = vObject.at(idx);
		*target = vObject.at(idx+1);

		PCLPointCloudPtr output_cloud(new PCLPointCloud);
		pcl::NormalDistributionsTransform<PCLPoint, PCLPoint> ndt;
		ndt.setTransformationEpsilon(0.00001);
		ndt.setStepSize (0.1);
		ndt.setResolution (0.005);
		ndt.setMaximumIterations (500);
		ndt.setInputSource(source);
		ndt.setInputTarget(target);
		ndt.align (*output_cloud);

		std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
			<< " score: " << ndt.getFitnessScore () << std::endl;

		Eigen::Matrix4f ftransform = ndt.getFinalTransformation();

		*newCloud = *object;
		pcl::transformPointCloud (*object, *newCloud, ftransform);
		*object = *newCloud;

		*object += *target;
		object->header.frame_id = "camera_rgb_optical_frame";
		d_cloud_pub.publish(object);
	}

	ROS_INFO_STREAM("Removing outliers");
	ROS_INFO_STREAM("Size before: " << object->size());
	PCLPointCloudPtr objectFiltered(new PCLPointCloud());
	// Create the filtering object
  	pcl::StatisticalOutlierRemoval<PCLPoint> sor;
	sor.setInputCloud(object);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.filter(*objectFiltered);

	ROS_INFO_STREAM("Size after: " << objectFiltered->size());

	PCLPointCloudPtr tempCloud(new PCLPointCloud());
	
	pcl::VoxelGrid<PCLPoint> vox;
  	vox.setInputCloud (objectFiltered);
  	vox.setLeafSize (0.002f, 0.002f, 0.002f);
  	vox.filter (*objectFiltered);

	ROS_INFO_STREAM("Size after after: " << objectFiltered->size());
	objectFiltered->header.frame_id = "camera_rgb_optical_frame";
	d_cloud_pub.publish(objectFiltered);

	ROS_INFO_STREAM("DONE!");
		
}