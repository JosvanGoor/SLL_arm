#include "simpleFeature.h"

void SimpleFeature::showImage()
{
	cv::imshow("Simple Feature Object detection", d_cloudImage);
	cv::waitKey(200);
}