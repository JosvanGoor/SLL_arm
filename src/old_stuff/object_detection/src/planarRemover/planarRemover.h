#ifndef H_PLANARREMOVER
#define H_PLANARREMOVER


#include "../utils.h"


class PlanarRemover
{
//	GPU gpu;
	
	public: 
		PlanarRemover(){};
		PCLPointCloudPtr removePlanarSurface(PCLPointCloudPtr cloud);
		PCLPointCloudPtr customRemovePlanar(PCLPointCloudPtr cloud);
		PCLPointCloudPtr customRemovePlanarGPU(PCLPointCloudPtr cloud);
};

#endif
