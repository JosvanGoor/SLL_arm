#include "sift.h"

using namespace cv;

void Sift::init()
{
	DIR *dir;
	struct dirent *ent;
	
	// same objects 
	vector<string> objectNames;
/*	objectNames.push_back("arizona");
	objectNames.push_back("carbonara");
	objectNames.push_back("erdnusse");
	objectNames.push_back("go");
	objectNames.push_back("holsten");
	objectNames.push_back("kartoffelknodel");
	objectNames.push_back("kinderriegel");
	objectNames.push_back("largecup");
	objectNames.push_back("organgensaft");
	objectNames.push_back("palmolive");
	objectNames.push_back("polar-frisch");
	objectNames.push_back("schnucki");
	objectNames.push_back("smallcup");
	objectNames.push_back("sohlen-latex");
	objectNames.push_back("sponge");
	objectNames.push_back("taco");
	objectNames.push_back("tape");
	objectNames.push_back("ultimategel");
	objectNames.push_back("xylit");
*/
	objectNames.push_back("wirelessbox");

/*
	for (size_t idp = 0; idp < 200; ++idp)
		objectNames.push_back("honeyherb");
	for (size_t idp = 0; idp < 200; ++idp)
		objectNames.push_back("honeyherb");
*/
	for (size_t ido = 0; ido < objectNames.size(); ++ido)
	{
		stringstream path;
		path << "/home/rik/sudo/ros/catkin_ws/src/object_detection/objects/";
		path << objectNames.at(ido) << "/";

	//	stringstream pcObject;
	//	pcObject << path.str() << "pcModels/" << objectNames.at(ido) << ".pcd";
	//	float volumeFront = getVolume(pcObject.str());

	//	stringstream pcObjectSide;
	//	pcObjectSide << path.str() << "pcModels/" << objectNames.at(ido) << "Side.pcd"; 
	//	float volumeSide = getVolume(pcObjectSide.str()); 
		
		path << "imageModels/";

		if ((dir = opendir (path.str().c_str())) != NULL) 
		{
		  	while ((ent = readdir (dir)) != NULL) 
		  	{
		  		stringstream fileName;
		  		fileName << ent->d_name;

		    	if (fileName.str() != "." && fileName.str() != "..")
		    	{
		    		Object object;
					Mat model;
					stringstream objectPath; 
					objectPath << path.str() << ent->d_name;
					
					model = imread(objectPath.str());
					
					CV_Assert(!model.empty());

					object.objectName = objectNames.at(ido);
					object.image =  model.clone();

					// if objects front volume is larger then threshold then return true, else false
					object.largeObject = true; 

					// SIFT feature detector and feature extractor
					cv::SiftFeatureDetector detector;
					cv::SiftDescriptorExtractor extractor;
		
					detector.detect(object.image, object.keypoints);

					// only accept models with more then 1 keypoint
					if (object.keypoints.size() >= 1)
					{
						extractor.compute( object.image, object.keypoints, object.descriptors);
						objects.push_back(object);
					}

		    	}
		  	}

		  	closedir(dir);
		} 
		else 
		{
		  /* could not open directory */
	  		perror ("could not open directory");
		}
	}

	cout << "Object models loaded: " << objects.size() << "\n";
      
}