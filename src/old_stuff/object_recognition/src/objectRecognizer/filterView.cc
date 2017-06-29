#include "objectRecognizer.ih"

/*
 * Contains:
 * filterView: crops the pointcloud from the center to the desired width
 */

// crops the pointcloud from the center to the desired width
PCLPointCloudPtr ObjectRecognizer::filterView(PCLPointCloudPtr input, double width, double depth)
{
	//create the filter and set the input
	PCLPointCloudPtr output(new PCLPointCloud);
	PCLPointCloudPtr throughput(new PCLPointCloud);
	PassThrough<Point> pass;
	pass.setInputCloud (input);

	//Filter only along the x-axis
	pass.setFilterFieldName ("x");
	//Set filter width
	pass.setFilterLimits (-0.5 * width, 0.5 * width);

	pass.setFilterFieldName ("y");
	//Set filter width
	pass.setFilterLimits (-1, 0);

	//Do the actual filtering
	pass.filter (*throughput);

	//Do the same for the z axis
	pass.setInputCloud(throughput);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-0.05,0.4);
	pass.filter(*output);

	return output;
}
