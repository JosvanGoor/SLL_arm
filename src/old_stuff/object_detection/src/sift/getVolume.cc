#include "sift.h"

float Sift::getVolume(string path)
{
	PCLPointCloud cloud;
	io::loadPCDFile<Point>(path, cloud);

	Point minPt, maxPt; 
	getMinMax3D(cloud, minPt, maxPt);

	float w,h,d; 

	// calculate width, height, depth
	w = sqrt(pow(maxPt.x - minPt.x, 2));
	h = sqrt(pow(maxPt.y - minPt.y, 2));
	d = sqrt(pow(maxPt.z - minPt.z, 2));

	return (float)w * 100 * h * 100 * d * 100;
}

float Sift::getVolume(PCLPointCloud &cloud)
{
	Point minPt, maxPt; 
	getMinMax3D(cloud, minPt, maxPt);

	float w,h,d; 

	// calculate width, height, depth
	w = sqrt(pow(maxPt.x - minPt.x, 2));
	h = sqrt(pow(maxPt.y - minPt.y, 2));
	d = sqrt(pow(maxPt.z - minPt.z, 2));

	return (float)w * 100 * h * 100 * d * 100;
}