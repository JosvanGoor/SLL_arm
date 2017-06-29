#include "objectDetection.h"
#include <sstream>
#include <pcl/io/png_io.h>
#include <cmath>


int64 workbegin = 0;
int64 workend = 0;

static void timeBegin()
{
	workbegin = cv::getTickCount();
}

static void timeEnd()
{
	workend = cv::getTickCount() - workbegin;
}

static double getTimeNow()
{
	return workend /((double)cvGetTickFrequency() * 1000.);
}


void ObjectDetection::pcCallBack(const sensor_msgs::PointCloud2ConstPtr &cloudMsg)
{	
	

	if (true) // needs to be run()
	{
		PCLPointCloudPtr receivedCloud(new PCLPointCloud); 
		PCLPointCloud unmodCloud;
		
		fromROSMsg(*cloudMsg, *receivedCloud); // convertion from ROS to PCL formats
		fromROSMsg(*cloudMsg, unmodCloud); // convertion from ROS to PCL formats
				
		// Remove planar surface
		PCLPointCloudPtr surflessCloud(new PCLPointCloud); 
		surflessCloud->header.frame_id = "camera_link";

		timeBegin(); // start timing 
		double time = 0.;
		surflessCloud = planarRemover.removePlanarSurface(receivedCloud);
		// only detect object that are on the surface? 	
		timeEnd();
		time = getTimeNow();
		std::cout << "Total run time: " << time << " ms" <<"\n";

	
	//	surflessCloud = receivedCloud;

		// filter on an area (TEMP), should filter on surface size...
		PCLPointCloudPtr pclCloud_filtered (new PCLPointCloud);
	//	pclCloud_filtered->height = surflessCloud->height;
	//	pclCloud_filtered->width = surflessCloud->width;
		pcl::PassThrough<Point> passthrough_filter;
		
		// filter X
/*		
		passthrough_filter.setInputCloud(surflessCloud);
	//	passthrough_filter.setKeepOrganized(true); 
		passthrough_filter.setFilterFieldName("x");
		passthrough_filter.setFilterLimits(-0.20, 0.25);
	//	passthrough_filter.setFilterLimitsNegative (true);
		passthrough_filter.filter(*surflessCloud);
		
		// filter Y
		passthrough_filter.setInputCloud(surflessCloud);
	//	passthrough_filter.setKeepOrganized(true); 
		passthrough_filter.setFilterFieldName("y");
		passthrough_filter.setFilterLimits(-0.10, 0.25);
	//	passthrough_filter.setFilterLimitsNegative (true);
		passthrough_filter.filter(*surflessCloud);
*/
		// filter Z
		passthrough_filter.setInputCloud(surflessCloud);
	//	passthrough_filter.setKeepOrganized(true); 
		passthrough_filter.setFilterFieldName("z");
		passthrough_filter.setFilterLimits(0, 1.2);
	//	passthrough_filter.setFilterLimitsNegative (true);
		passthrough_filter.filter(*surflessCloud);	


		vector<pcl::PointIndices> clusters;
		
		if (surflessCloud->points.size() > 0)		
			clusters = clusterObjects.cluster(surflessCloud); // cluster objects


		// grab individual objects
		if (clusters.size() == 0)
		{
			ROS_INFO("Did not find any clusters");
		}
		else
		{			
			PCLPointCloudPtr pclCloudToPublish (new PCLPointCloud);
			pclCloudToPublish->header.frame_id = "camera_link";
			cout << "Number of clusters found: " << clusters.size() << "\n";		
			
			simpleFeature.makeImage(unmodCloud);		

			vector<DetectedObject> objectsDetected; 

		//	#pragma omp parallel for ordered schedule(dynamic)
			for (size_t idc = 0; idc < clusters.size(); ++idc)
			{			
				PCLPointCloudPtr pclCloud_segmented (new PCLPointCloud);
				pclCloud_segmented->header.frame_id = "camera_link";
				pcl::PointIndices point_indices = clusters.at(idc);


				foreach (int index, point_indices.indices)			
				{
					Point p = surflessCloud->points[index];				
					pclCloud_segmented->points.push_back(p);
					pclCloudToPublish->points.push_back(p);
				}  			
				
				simpleFeature.setObjectCloud(*pclCloud_segmented);
			//	simpleFeature.match();	
				ObjectWindow objectWindow = simpleFeature.getObjectWindow();
				cv::Mat cloudImage = simpleFeature.getImage();
				
				cv::Rect roi(cv::Point(objectWindow.xLeft, objectWindow.yTop), 
				cv::Point(objectWindow.xRight, objectWindow.yBottom));
				
			
				float volume = sift.getVolume(*pclCloud_segmented);
				set<string> objectsFound = sift.match(cloudImage(roi).clone(), volume > 1600 ? true : false);	
		
				
				if (objectsFound.size() > 0)
				{
					for (set<string>::iterator idof = objectsFound.begin(); idof != objectsFound.end(); ++idof)
					{
						Point minPt;
						Point maxPt;

						minPt = objectWindow.minPt;
						maxPt = objectWindow.maxPt;

						DetectedObject object;
						object.objectName = *idof;

						// take the average x, y, z point from the clustered object
						object.x = (minPt.x + maxPt.x) / 2;
						object.y = (minPt.y + maxPt.y) / 2;
						object.z = (minPt.z + maxPt.z) / 2;

						objectsDetected.push_back(object);
					}
					
				}
			/*	else // an unknown object/cluster is found? 
				{
						Point minPt;
						Point maxPt;

						minPt = objectWindow.minPt;
						maxPt = objectWindow.maxPt;
						DetectedObject object;
						object.objectName = "Unknown";

						// take the average x, y, z point from the clustered object
						object.x = (minPt.x + maxPt.x) / 2;
						object.y = (minPt.y + maxPt.y) / 2;
						object.z = (minPt.z + maxPt.z) / 2;

						objectsDetected.push_back(object);
				}
			*/	
				

		


			/*
				pclCloud_segmented->width = pclCloud_segmented->points.size();
				pclCloud_segmented->height = 1;

				stringstream objectPath;
				objectPath << "/home/rik/sudo/ros/objectDetection/object_" << idc << ".pcd";
				io::savePCDFile(objectPath.str(), *pclCloud_segmented);	
			*/	
			}


			writeToMemory(objectsDetected);
		//	cout << "current: " << d_currentSampleNr << ", " << d_numberOfSamples << "\n";
			if (d_currentSampleNr == d_numberOfSamples - 1)
				writeStop();
			else
				++d_currentSampleNr;
			
		//	simpleFeature.showImage();	
			
	//		cloud_pub.publish(*pclCloudToPublish);
			
		}
		
		cloud_pub.publish(*surflessCloud);
	}

	// temp code for timing only
	
}
