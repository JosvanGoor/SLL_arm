#include "simpleFeature.h"

void SimpleFeature::initNormalFeature()
{
	/*for (size_t idc = 0; idc < d_simpleFeatures.size(); ++idc)
	{
		pcl::NormalEstimationOMP<Point, pcl::Normal> ne;
		ne.setInputCloud(d_simpleFeatures.at(idc).objectCloud.makeShared());
		
		pcl::search::KdTree<Point>::Ptr searchTree (new pcl::search::KdTree<Point>());
		ne.setSearchMethod(searchTree);
		
		pcl::PointCloud<pcl::Normal>::Ptr cloudNormals (new pcl::PointCloud<pcl::Normal>);
		
		ne.setRadiusSearch(0.01);
		ne.compute(*cloudNormals);
		
		d_simpleFeatures.at(idc).objectNormals = *cloudNormals;
		
		PCLPointCloud cloud = d_simpleFeatures.at(idc).objectCloud;
		
		Point featurePoint;
		
		for (size_t idx = 0; idx < cloud.size(); ++idx)
		{
			float x = pow(d_simpleFeatures.at(idc).meanPoint.x - cloud.points.at(idx).x, 2);
			float y = pow(d_simpleFeatures.at(idc).meanPoint.y - cloud.points.at(idx).y, 2);
			float z = pow(d_simpleFeatures.at(idc).meanPoint.z - cloud.points.at(idx).z, 2);
			
			float dist = sqrt(x + y + z);
			
			if (dist == d_simpleFeatures.at(idc).minDistance)
			{
				featurePoint = cloud.points.at(idx);
				break;
			}
		}
		
		pcl::search::KdTree<Point> tree;
		tree.setInputCloud(cloud.makeShared());	
		
		vector<int> pointIdxRadiusSearch;
		vector<float> pointRadiusSquaredDistance;
		float radius = (float)0.01f;
		
		tree.radiusSearch (featurePoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
		
		float lowestX = cloud.points.at(pointIdxRadiusSearch.at(0)).x;
		size_t lowestXid;
		
		float highestX = cloud.points.at(pointIdxRadiusSearch.at(0)).x;
		size_t highestXid;

		float lowestY = cloud.points.at(pointIdxRadiusSearch.at(0)).y;
		size_t lowestYid;

		float highestY = cloud.points.at(pointIdxRadiusSearch.at(0)).y;
		size_t highestYid;
		
		for (size_t idr = 0; idr < pointIdxRadiusSearch.size(); ++idr)
		{
			if (cloud.points.at(pointIdxRadiusSearch.at(idr)).x < lowestX)
			{
				lowestX = cloud.points.at(pointIdxRadiusSearch.at(idr)).x;
				lowestXid = pointIdxRadiusSearch.at(idr);
			}
			
			if (cloud.points.at(pointIdxRadiusSearch.at(idr)).x > highestX)
			{
				highestX = cloud.points.at(pointIdxRadiusSearch.at(idr)).x;
				highestXid = pointIdxRadiusSearch.at(idr);
			}

			if (cloud.points.at(pointIdxRadiusSearch.at(idr)).y < lowestY)
			{
				lowestY = cloud.points.at(pointIdxRadiusSearch.at(idr)).y;
				lowestYid = pointIdxRadiusSearch.at(idr);
			}

			if (cloud.points.at(pointIdxRadiusSearch.at(idr)).y > highestY)
			{
				highestY = cloud.points.at(idr).y;
				highestYid = pointIdxRadiusSearch.at(idr);
			}
		}
		
		d_simpleFeatures.at(idc).normalFeatures.push_back(cloudNormals->points.at(lowestXid));
		d_simpleFeatures.at(idc).normalFeatures.push_back(cloudNormals->points.at(highestXid));
		d_simpleFeatures.at(idc).normalFeatures.push_back(cloudNormals->points.at(lowestYid));
		d_simpleFeatures.at(idc).normalFeatures.push_back(cloudNormals->points.at(highestYid));
	
	}*/	
}