#include "objectRecognizer.ih"

/*
 * Contains:
 * getSizes: computes the heights, widths and positions for each cluster for a set of clusters
 * getExtremes: computes the extreme coordinates of a given cluster
 */

//computes the heights, widths and positions for each cluster for a set of clusters
void ObjectRecognizer::getSizes(vector<PCLPointCloudPtr> inputCld)
{
	d_inputSizes.clear();
	d_inputPositions.clear();

	vector<double> extremes;
	vector<double> size;
	vector<double> position;
	//for all clusters, compute the height and width and store it
	for(size_t idx = 0; idx != inputCld.size(); ++idx)
	{
		extremes = getExtremes(inputCld[idx]);
		size.clear();
		size.push_back(extremes[1] - extremes[0]); 	//max_x - min_x
		size.push_back(extremes[3] - extremes[2]); 	//max_y - min_y
		d_inputSizes.push_back(size);

		position.clear();
		position.push_back((extremes[1] + extremes[0]) / 2); 	// middle over x axis
		position.push_back((extremes[3] + extremes[2]) / 2); 	// middle over y axis
		position.push_back(extremes[5]);					 	// max of z axis
		position.push_back(extremes[5] - extremes[4]); // the height of the object, maxZ - minZ = height
		d_inputPositions.push_back(position);
	}

}

//computes the extreme coordinates of a given cluster
vector<double> ObjectRecognizer::getExtremes(PCLPointCloudPtr inputCld)
{
	//initialize min and max values for the x and y axis
	double min_x = DBL_MAX, min_y = DBL_MAX, min_z = DBL_MAX;
	double max_x = -DBL_MAX, max_y = -DBL_MAX, max_z = -DBL_MAX;

	//create return variable
	vector<double> of_the_jedi;

	//check the position of all points in the cluster...
	for (size_t idx = 0; idx != inputCld->points.size(); ++idx)
	{	
		//first check over the x axis..
		if (inputCld->points[idx].x < min_x)
			min_x = inputCld->points[idx].x;
		if (inputCld->points[idx].x > max_x)
			max_x = inputCld->points[idx].x;

		//then over the y axis..
		if (inputCld->points[idx].y < min_y)
			min_y = inputCld->points[idx].y;
		if (inputCld->points[idx].y > max_y)
			max_y = inputCld->points[idx].y;

		//and over the z axis..
		if (inputCld->points[idx].z < min_z)
			min_z = inputCld->points[idx].z;
		if (inputCld->points[idx].z > max_z)
			max_z = inputCld->points[idx].z;
	}

	//store the min and max values
	of_the_jedi.push_back(min_x);
	of_the_jedi.push_back(max_x);
	of_the_jedi.push_back(min_y);
	of_the_jedi.push_back(max_y);
	of_the_jedi.push_back(min_z);
	of_the_jedi.push_back(max_z);

	return of_the_jedi;
}
