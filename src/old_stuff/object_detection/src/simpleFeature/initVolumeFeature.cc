#include "simpleFeature.h"

void SimpleFeature::initVolumeFeature()
{
	for (size_t ido = 0; ido < d_simpleFeatures.size(); ++ido)
	{
		SimpleFeatures simpleFeature = d_simpleFeatures.at(ido);

		Point minPt, maxPt;  

		getMinMax3D(simpleFeature.objectCloud, minPt, maxPt);

		float w,h,d; 

		w = sqrt(pow(maxPt.x - minPt.x, 2));
		h = sqrt(pow(maxPt.y - minPt.y, 2));
		d = sqrt(pow(maxPt.z - minPt.z, 2));

		cout << simpleFeature.objectName << ": " << w << ", " << h << ", " << d << "\n";
		cout << w * 100 * h * 100* d * 100 << "\n";

		d_simpleFeatures.at(ido).volume = w * 100 * h * 100* d * 100;
	}
}