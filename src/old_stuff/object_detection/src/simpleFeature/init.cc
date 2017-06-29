#include "simpleFeature.h"

void SimpleFeature::init()
{
	vector<string> objectNames;
//	objectNames.push_back("bluebox");
//	objectNames.push_back("blueboxSide");
//	objectNames.push_back("fanbox");
//	objectNames.push_back("fanboxSide");
//	objectNames.push_back("wirelessbox");
//	objectNames.push_back("wirelessboxSide");
//	objectNames.push_back("honeyherb");
//	objectNames.push_back("honeyherbSide");

	
	for (size_t ido = 0; ido < objectNames.size(); ++ido)
	{
		stringstream objectName;
		objectName << "/home/rik/sudo/ros/objectDetection/objects/" << objectNames.at(ido) << ".pcd";
		pcl::PointCloud<Point> objectCloud;
	
		io::loadPCDFile<Point>(objectName.str(), objectCloud);
			
		float meanX = 0.0f;
		float meanY = 0.0f;
		float meanZ = 0.0f;
		
		for (size_t idx = 0; idx < objectCloud.points.size(); ++idx)
		{
			meanX += objectCloud.points.at(idx).x;
			meanY += objectCloud.points.at(idx).y;
			meanZ += objectCloud.points.at(idx).z;
		}
		
		meanX /=  objectCloud.points.size();
		meanY /=  objectCloud.points.size();
		meanZ /=  objectCloud.points.size();
		
		vector<float> distance;
		
		for (size_t idx = 0; idx < objectCloud.points.size(); ++idx)
		{
			float x = pow(meanX - objectCloud.points.at(idx).x, 2);
			float y = pow(meanY - objectCloud.points.at(idx).y, 2);
			float z = pow(meanZ - objectCloud.points.at(idx).z, 2);
			
			float dist = sqrt(x + y + z);
			distance.push_back(dist);
		}
		
		sort(distance.begin(), distance.end());
		
		
		float meanDist = 0;
		
		for (size_t idx = 0; idx < distance.size(); ++idx)
		{
			meanDist += distance.at(idx);
		}
	
		meanDist /= distance.size();		

		SimpleFeatures simpleFeatures;
		
		Point pt;
		pt.x = meanX;
		pt.y = meanY;
		pt.z = meanZ;
		
		simpleFeatures.objectName = objectNames.at(ido);
		simpleFeatures.minDistance = distance.at(0);
		simpleFeatures.maxDistance = distance.at(distance.size() - 1);
		simpleFeatures.meanDistance = meanDist;
		simpleFeatures.cloudSize = objectCloud.points.size();
		simpleFeatures.objectCloud = objectCloud;
		simpleFeatures.meanPoint = pt;
		
		d_simpleFeatures.push_back(simpleFeatures);
	
	}
	
	initVolumeFeature();
	initNormalFeature();
}