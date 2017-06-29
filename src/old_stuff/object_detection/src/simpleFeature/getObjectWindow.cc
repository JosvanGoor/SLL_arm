#include "simpleFeature.h"

ObjectWindow SimpleFeature::getObjectWindow()
{
	ObjectWindow objectWindow;
	
	Point minPt, maxPt;
	getMinMax3D(d_objectCloud, minPt, maxPt);	
	int left = 0;
	int right = 0;
	int top = 0;
	int bottom = 0;
	
	for (size_t idx = 0; idx < d_unmodCloud.points.size(); ++idx)
	{
		Point p = d_unmodCloud.points[idx];
		if (minPt.x == p.x)
			left = idx;
		
		if (minPt.y == p.y)
			bottom = idx;
		
		if (maxPt.x == p.x)
			right = idx;
		
		if (maxPt.y == p.y)
			top = idx;
		
		if (left != 0 && right != 0 &&
			top != 0 && bottom != 0)
			break;		
	}
	
	size_t xLeft, xRight, yTop, yBottom;			

	int div = d_unmodCloud.width;
	int bufferSize = 2;
	xLeft = left % div - bufferSize < 0 ? 0 : left % div - bufferSize ;
	xRight =  right % div + bufferSize > 640 ? 639 : right % div + bufferSize;
	
	yBottom = ceil(top / div) + bufferSize > 480 ? 479 : ceil(top / div) + bufferSize;
	yTop = ceil(bottom / div) - bufferSize < 0 ? 0 : ceil(bottom / div) - bufferSize;
	
	objectWindow.xLeft = xLeft;
	objectWindow.xRight = xRight;
	objectWindow.yTop = yTop;
	objectWindow.yBottom = yBottom;
	objectWindow.minPt = minPt;
	objectWindow.maxPt = maxPt;
	
	return objectWindow;
}