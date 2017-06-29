#include "simpleFeature.h"

void SimpleFeature::match()
{
/*	float meanX = 0.0f;
	float meanY = 0.0f;
	float meanZ = 0.0f;
	
	size_t sizeObject =  d_objectCloud.points.size();
	
	for (size_t idx = 0; idx < sizeObject; ++idx)
	{
		meanX += d_objectCloud.points.at(idx).x;
		meanY += d_objectCloud.points.at(idx).y;
		meanZ += d_objectCloud.points.at(idx).z;
	}	
	
	meanX /=  sizeObject;
	meanY /=  sizeObject;
	meanZ /=  sizeObject;
	
	vector<float> distance;
	
	for (size_t idx = 0; idx < sizeObject; ++idx)
	{
		float x = pow(meanX - d_objectCloud.points.at(idx).x, 2);
		float y = pow(meanY - d_objectCloud.points.at(idx).y, 2);
		float z = pow(meanZ - d_objectCloud.points.at(idx).z, 2);
		
		float dist = sqrt(x + y + z);
		distance.push_back(dist);
	}
	
	sort(distance.begin(), distance.end());	
	
	float minDist = distance.at(0);

	pcl::NormalEstimationOMP<Point, pcl::Normal> ne;
	ne.setInputCloud(d_objectCloud.makeShared());
	
	pcl::search::KdTree<Point>::Ptr searchTree (new pcl::search::KdTree<Point>());
	ne.setSearchMethod(searchTree);
	
	pcl::PointCloud<pcl::Normal>::Ptr cloudNormals (new pcl::PointCloud<pcl::Normal>);
	
	ne.setRadiusSearch(0.01);
	ne.compute(*cloudNormals);	
	
	Point featurePoint;

	for (size_t idx = 0; idx < d_objectCloud.size(); ++idx)
	{
		float x = pow(meanX - d_objectCloud.points.at(idx).x, 2);
		float y = pow(meanY - d_objectCloud.points.at(idx).y, 2);
		float z = pow(meanZ - d_objectCloud.points.at(idx).z, 2);
		
		float dist = sqrt(x + y + z);
		
		if (dist == minDist)
		{
			featurePoint = d_objectCloud.points.at(idx);
			break;
		}
	}

	pcl::search::KdTree<Point> tree;
	tree.setInputCloud(d_objectCloud.makeShared());	
	
	vector<int> pointIdxRadiusSearch;
	vector<float> pointRadiusSquaredDistance;
	float radius = (float)0.01f;
	
	tree.radiusSearch(featurePoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

	float lowestX = d_objectCloud.points.at(pointIdxRadiusSearch.at(0)).x;
	size_t lowestXid;
	
	float highestX = d_objectCloud.points.at(pointIdxRadiusSearch.at(0)).x;
	size_t highestXid;

	float lowestY = d_objectCloud.points.at(pointIdxRadiusSearch.at(0)).y;
	size_t lowestYid;

	float highestY = d_objectCloud.points.at(pointIdxRadiusSearch.at(0)).y;
	size_t highestYid;
	
	for (size_t idr = 0; idr < pointIdxRadiusSearch.size(); ++idr)
	{
	//	d_objectCloud.points.at(pointIdxRadiusSearch.at(idr)).r = 255;
	//	d_objectCloud.points.at(pointIdxRadiusSearch.at(idr)).g = 0;
	//	d_objectCloud.points.at(pointIdxRadiusSearch.at(idr)).b = 0;

		if (d_objectCloud.points.at(pointIdxRadiusSearch.at(idr)).x < lowestX)
		{
			lowestX = d_objectCloud.points.at(pointIdxRadiusSearch.at(idr)).x;
			lowestXid = pointIdxRadiusSearch.at(idr);
		}
		
		if (d_objectCloud.points.at(pointIdxRadiusSearch.at(idr)).x > highestX)
		{
			highestX = d_objectCloud.points.at(pointIdxRadiusSearch.at(idr)).x;
			highestXid = pointIdxRadiusSearch.at(idr);
		}

		if (d_objectCloud.points.at(pointIdxRadiusSearch.at(idr)).y < lowestY)
		{
			lowestY = d_objectCloud.points.at(pointIdxRadiusSearch.at(idr)).y;
			lowestYid = pointIdxRadiusSearch.at(idr);
		}

		if (d_objectCloud.points.at(pointIdxRadiusSearch.at(idr)).y > highestY)
		{
			highestY = d_objectCloud.points.at(pointIdxRadiusSearch.at(idr)).y;
			highestYid = pointIdxRadiusSearch.at(idr);
		}
	}

	Point pt = d_objectCloud.points.at(lowestXid);
	pt.x += cloudNormals->points.at(lowestXid).normal_x;
	pt.y += cloudNormals->points.at(lowestXid).normal_y;
	pt.z += cloudNormals->points.at(lowestXid).normal_z;
	pt.r = 255;
	pt.g = 0;
	pt.b = 0;
	d_objectCloud.points.push_back(pt);

	pt = d_objectCloud.points.at(lowestYid);
	pt.x += cloudNormals->points.at(lowestYid).normal_x;
	pt.y += cloudNormals->points.at(lowestYid).normal_y;
	pt.z += cloudNormals->points.at(lowestYid).normal_z;
	pt.r = 255;
	pt.g = 0;
	pt.b = 255;
	d_objectCloud.points.push_back(pt);

	d_objectCloud.points.at(lowestXid).r = 255;
	d_objectCloud.points.at(lowestXid).g = 0;
	d_objectCloud.points.at(lowestXid).b = 0;

	d_objectCloud.points.at(highestXid).r = 0;
	d_objectCloud.points.at(highestXid).g = 255;
	d_objectCloud.points.at(highestXid).b = 0;

	d_objectCloud.points.at(highestYid).r = 0;
	d_objectCloud.points.at(highestYid).g = 255;
	d_objectCloud.points.at(highestYid).b = 255;

	d_objectCloud.points.at(lowestYid).r = 255;
	d_objectCloud.points.at(lowestYid).g = 0;
	d_objectCloud.points.at(lowestYid).b = 255;

	publisher.publish(d_objectCloud);

	pcl::Normal lnX, hnX, lnY, hnY;

	lnX = cloudNormals->points.at(lowestXid);
	lnY = cloudNormals->points.at(lowestYid);
	hnX = cloudNormals->points.at(highestXid);
	hnY = cloudNormals->points.at(highestYid);

	vector<pcl::Normal> normalFeatures;
	normalFeatures.push_back(lnX);
	normalFeatures.push_back(hnX);
	normalFeatures.push_back(lnY);
	normalFeatures.push_back(hnY);

	for (size_t idf = 0; idf < d_simpleFeatures.size(); ++idf)
	{
		vector<pcl::Normal> tempNormalFeatures = d_simpleFeatures.at(idf).normalFeatures;

		float dist = 0;

		for (size_t idn = 0; idn < tempNormalFeatures.size(); ++idn)
		{
			dist += pow(tempNormalFeatures.at(idn).normal_x - normalFeatures.at(idn).normal_x, 2) +
					pow(tempNormalFeatures.at(idn).normal_y - normalFeatures.at(idn).normal_y, 2) +
					pow(tempNormalFeatures.at(idn).normal_z - normalFeatures.at(idn).normal_z, 2);

		}

		dist = sqrt(dist);
		cout << d_simpleFeatures.at(idf).objectName << ": " << dist << '\n';
	}


/*	
	for (size_t ido = 0; ido < d_simpleFeatures.size(); ++ido)
	{
		SimpleFeatures simpleFeatures = d_simpleFeatures.at(ido);
		
		float euDist = sqrt(pow(minDist - simpleFeatures.minDistance, 2) +
							pow(meanDist - simpleFeatures.meanDistance, 2) +
							pow(maxDist - simpleFeatures.maxDistance, 2));
		cout << simpleFeatures.objectName <<  ": " << euDist << "\n";
		
		if (euDist < 0.001) // threshold value
		{
			ObjectWindow objectWindow = getObjectWindow();
			cv::rectangle(d_cloudImage, cv::Point(objectWindow.xLeft, objectWindow.yTop), cv::Point(objectWindow.xRight, objectWindow.yBottom), cv::Scalar(255,0,0));
			cv::putText(d_cloudImage, simpleFeatures.objectName, cv::Point(objectWindow.xLeft, objectWindow.yTop - 10), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 0, 0));
		}
	}
*/
}