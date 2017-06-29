#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <sstream>
#include <iostream>
#include <fstream>

// Borg Memory service
#include <borg_pioneer/MemoryReadSrv.h>
#include <borg_pioneer/MemorySrv.h>

// PointCloud includes
#include <sensor_msgs/PointCloud2.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/passthrough.h>
///#include <pcl_ros/conversions.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/filters/passthrough.h>
//#include <pcl/io/png_io.h>
#include <pcl_conversions/pcl_conversions.h>

// Clustering
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using boost::property_tree::ptree;

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PCLPointCloud;
typedef pcl::PointCloud<Point>::Ptr PCLPointCloudPtr;

const int FILTERING =       0;
const int CLUSTERING =      1;
const int PICTURES =        2;  // Both filtering and clustering

#ifdef __CDT_PARSER__
    #define foreach(a, b) for(a : b)
#else
    #define foreach(a, b) BOOST_FOREACH(a, b)
#endif

// Variables to keep track of data streams
bool camera_data[] = {false, false};
PCLPointCloudPtr camera_cloud1;
PCLPointCloudPtr camera_cloud2;

// For publishing PointCloud data. We don't want to create a new publisher every time
ros::Publisher personCloud_pub;

// Memory service client
ros::ServiceClient memory_client;
ros::ServiceClient memory_client_write;

// If you declare a listener in the callback function, then it loses the history of the tf data.
// It therefore starts nagging about "Can't find frame_id 'base_link'" and stuff. So a global listener is required.
// The listener can't be initialized here, as ros::init has to be called first.
tf::TransformListener* listener;

struct Human
{
	float x;
	float y;
	float z;
	float x_pix;
	float y_pix;
	std::vector<Point> points;
    
    // Features
    double height;
    double legs_red;
    double legs_green;
    double legs_blue;
    double legs_white;
    double legs_black;
    double legs_grey;
    double upper_red;
    double upper_green;
    double upper_blue;
    double upper_white;
    double upper_black;
    double upper_grey;
};

struct rgb
{
    float r;
    float g;
    float b;
};

class SegmentHumans
{
//Data members
private:
	std::vector<Human*> d_humans;	// Human information
	std::string d_base_frame;		// Base frame string (e.g. "/base_link")
	double d_human_width;			// Approx. width of a human
	double d_human_depth;			// Approx. depth of a human
	double d_human_height;          // Approx. height of a human
	bool d_publish_cloud;			// Publish segmented humans (Y/N)
	int mode;                       // What mode of data analysis and output should be used
	int image_counter;
	int max_images;
	int gap_limit;                  // Size of allowed gap in array between person and object
	float image_ratio;              // Amount the image should be scaled by from cloud dimensions
	int image_width;                // The width of the image output
	int image_height;               // The height of the image output
	rgb *image_array_filt;          // The array that holds the colours of the image for the filtering method
	rgb *image_array_clust;         // The array that holds the colours of the image for the clustering method
	double frame_times[20];         // The time used to record the length of clustering and filtering methods 
	int frame_iterator;
	
	int max_fps;

//Functions
public:
    void initImageArrays();
	void readHumanDetectionsFromMemory(std::string message);
	void readFromMemory();
    void writeToMemory();
	void depthCallback(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);
	bool availabilityChecks(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);
	void processCam1(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);
	void processCam2(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);
	PCLPointCloudPtr getCloud(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);
	void findHumans(PCLPointCloudPtr pclCloud);
	void checkData();
    void getFeatures(PCLPointCloudPtr pclCloud, Human* human);
	void saveBMP(const char *filename, int w, int h, int dpi, rgb *data);

	SegmentHumans(bool publish_cloud)
		: d_base_frame("/base_link"),
		  d_human_width(2.0),
		  d_human_depth(1.2),
		  d_human_height(2.0),
		  mode(FILTERING),
		  image_counter(0),
		  max_images(8),
		  gap_limit(2),
		  //image_ratio(0.44),
		  image_width(200),
		  frame_iterator(0),
		  d_publish_cloud(publish_cloud),
		  max_fps(20)
	{
	    image_ratio = d_human_width / d_human_height;
	}
	~SegmentHumans()
	{
		foreach (Human* human, d_humans)
		{
			delete human;
		}
		delete image_array_filt;
		delete image_array_clust;
	};
};

void SegmentHumans::initImageArrays()
{
    image_height = (int)(image_width / image_ratio);    
    
    // Initialise arrays
    image_array_filt = new rgb[image_width*image_height];
    image_array_clust = new rgb[image_width*image_height];
}

void SegmentHumans::readHumanDetectionsFromMemory(std::string message)
{
	// Clear humans
	foreach (Human* human, d_humans)
	{
		delete human;	// DIE!!!!
	}
	d_humans.clear();

	// Check if there is any data
	if (message.size() == 0 || strcmp(message.c_str(), "null") == 0)
	{
		// No data
		return;
	}

	//Get rid of unwanted info
	message = message.substr(1, message.size() - 2);		// Remove outer brackets
	size_t first_occurrence_opening_bracket = message.find_first_of('{');
	size_t last_occurrence_closing_bracket = message.find_last_of('}');
	message = message.substr(first_occurrence_opening_bracket, last_occurrence_closing_bracket - first_occurrence_opening_bracket + 1);

	// Separate responses, message can contain multiple responses
	std::vector<std::string> responses;
	boost::char_separator<char> sep("}");
	boost::tokenizer< boost::char_separator<char> > tokens(message, sep);
	int count = 0;
	for (boost::tokenizer<boost::char_separator<char> >::iterator it = tokens.begin(); it != tokens.end(); ++it)
	{
		std::string single_response = *it;
		single_response += '}';	// Add the closing bracket that was first removed by splitting the response
		if (count != 0) single_response = single_response.substr(2, single_response.size() - 2);	// Remove seperating comma between single responses
		responses.push_back(single_response);
		++count;
	}

	// Decode data
	foreach (std::string response, responses)
	{
		ptree pt;
		std::stringstream ss; ss << response;
		boost::property_tree::read_json(ss, pt);

		// Check timestamp
		double stamp = pt.get<double>("time");
		double now = ros::Time::now().toSec();
		if ((now - stamp) > 0.2)
		{
			// This observation was too long ago -> skip
			continue;
		}

		// Get human info and save
		Human* human = new Human();	// I have the power of God!
		human->x = pt.get<double>("x_actual");
		human->y = pt.get<double>("y_actual");
		human->z = pt.get<double>("z_actual");
		human->x_pix = pt.get<double>("x");
		human->y_pix = pt.get<double>("depth");
		d_humans.push_back(human);	// Sorry human ...
	}
}

void SegmentHumans::readFromMemory()
{
	// Create request
	borg_pioneer::MemoryReadSrv srv;
	srv.request.function = "get_last_observation";
	srv.request.timestamp = ros::Time::now();
	srv.request.name = "HumanDetector";
	srv.request.params = "";

	// Make the call
	memory_client.call(srv);

	// Read message
	readHumanDetectionsFromMemory(srv.response.json);
}


void SegmentHumans::writeToMemory()
{
	// Create JSON property dict, it will have the following form:
	// {
	// 		1: {'x': x_1, 'y': y_1, ...},	//Person 1
	//		2: {'x': x_2, 'y': y_2, ...}	//Person 2, etc.
	// }
	boost::property_tree::ptree pt_prop_dict;
    int person_nr = 1;
	foreach (Human* human, d_humans)
	{
		// Create person_dict
		boost::property_tree::ptree pt_person_dict;
		pt_person_dict.put("height", human->height);
		pt_person_dict.put("x", human->x);
		pt_person_dict.put("y", human->y);
		pt_person_dict.put("z", human->z);
		pt_person_dict.put("legs_black", human->legs_black);
		pt_person_dict.put("legs_white", human->legs_white);
		pt_person_dict.put("legs_grey", human->legs_grey);
		pt_person_dict.put("legs_red", human->legs_red);
		pt_person_dict.put("legs_green", human->legs_green);
		pt_person_dict.put("legs_blue", human->legs_blue);
		pt_person_dict.put("upper_black", human->upper_black);
		pt_person_dict.put("upper_white", human->upper_white);
		pt_person_dict.put("upper_grey", human->upper_grey);
		pt_person_dict.put("upper_red", human->upper_red);
		pt_person_dict.put("upper_green", human->upper_green);
		pt_person_dict.put("upper_blue", human->upper_blue);
		pt_person_dict.put("x_pix", static_cast<int>(human->x_pix));
		pt_person_dict.put("y_pix", static_cast<int>(human->y_pix));
		//ROS_INFO("----------legs:\n black: %f\n white: %f\n grey: %f\n red: %f\n green: %f\n blue: %f", human->legs_black, human->legs_white, human->legs_grey, human->legs_red, human->legs_green, human->legs_blue);
		std::ostringstream buf_person_dict;
		boost::property_tree::write_json(buf_person_dict, pt_person_dict, false);

		// Add person dict to property dict
		std::string person_nr_str = boost::lexical_cast<std::string>(person_nr);
		pt_prop_dict.put(person_nr_str, buf_person_dict.str());
		++person_nr;
	}
	std::ostringstream buf_prop_dict;
	boost::property_tree::write_json(buf_prop_dict, pt_prop_dict, false);
	std::string prop_dict = buf_prop_dict.str();

	// Output findings to memory
	borg_pioneer::MemorySrv srv;
	srv.request.name = "humans_found";
	srv.request.timestamp = ros::Time::now();
	srv.request.json = prop_dict;

	//ROS_INFO(prop_dict.c_str());

	// Make the call
	if (!memory_client_write.call(srv))
        ROS_WARN("Could not write to memory, damnit");

}

bool SegmentHumans::availabilityChecks(const sensor_msgs::PointCloud2ConstPtr &cloudMsg)
{
	// Read memory
	readFromMemory();

	// Check if there are humans detected
	if (d_humans.size() == 0)
	{
		//ROS_INFO("No humans detected");
		return false;
	}

	// Check if transform is available
	try
	{
		tf::StampedTransform transform;
		listener->lookupTransform(d_base_frame, cloudMsg->header.frame_id, cloudMsg->header.stamp,transform);
	}
	catch (tf::LookupException &ex)
	{
		// No transform available
		ROS_INFO("No transform available");
		return false;
	}
	return true;
}

PCLPointCloudPtr SegmentHumans::getCloud(const sensor_msgs::PointCloud2ConstPtr &cloudMsg)
{
	// Transform PointCloud to base frame
	sensor_msgs::PointCloud2 cloudMsg_transformed;
	pcl_ros::transformPointCloud(d_base_frame, *cloudMsg, cloudMsg_transformed, *listener);

	// Convert PointCloud2 to PCL::PointCloud
	PCLPointCloudPtr pclCloud (new PCLPointCloud);
	pcl::fromROSMsg(cloudMsg_transformed, *pclCloud);
	
	return pclCloud;
}

void SegmentHumans::processCam1(const sensor_msgs::PointCloud2ConstPtr &cloudMsg)
{
    //ROS_INFO("Received data from cam: 1");
    if(availabilityChecks(cloudMsg))
    { 
        camera_cloud1 = getCloud(cloudMsg);
        camera_data[0] = true;
    }
    checkData();
}

void SegmentHumans::processCam2(const sensor_msgs::PointCloud2ConstPtr &cloudMsg)
{
    //ROS_INFO("Received data from cam: 2"); 
    if(availabilityChecks(cloudMsg))
    { 
        camera_cloud2 = getCloud(cloudMsg);
        camera_data[1] = true;
    }
    checkData();
}

void SegmentHumans::findHumans(PCLPointCloudPtr pclCloud)
{
	// Read memory
	readFromMemory();

	// Discard floor using voxel grid filter
	PCLPointCloudPtr pclCloud_filtered (new PCLPointCloud);
	pcl::VoxelGrid<Point> voxelgrid_filter;
	voxelgrid_filter.setInputCloud(pclCloud);
	voxelgrid_filter.setLeafSize(0.01f, 0.01f, 0.01f);  // Leaf size: 1 cm
	voxelgrid_filter.setFilterFieldName("z");
	voxelgrid_filter.setFilterLimits(0.1, 5);			// Everything < 0.2m is considered floor
	voxelgrid_filter.filter(*pclCloud_filtered);

	// Create new PointCloud that contains the segmented persons
	// cloud for method 1
	PCLPointCloudPtr pclCloud_segmented_filt (new PCLPointCloud);
	//pclCloud_segmented_filt->header = pcl_conversions::toPCL(pclCloud_segmented_filt->header);
	pclCloud_segmented_filt->header.frame_id = d_base_frame;
	pclCloud_segmented_filt->header.stamp = ros::Time::now().toSec();
	pclCloud_segmented_filt->height = 1;
	pclCloud_segmented_filt->is_dense = true;
	// cloud for method 2
	PCLPointCloudPtr pclCloud_segmented_clust (new PCLPointCloud);
	pclCloud_segmented_clust->header.frame_id = d_base_frame;
	pclCloud_segmented_clust->header.stamp = ros::Time::now().toSec();
	pclCloud_segmented_clust->height = 1;
	pclCloud_segmented_clust->is_dense = true;

	foreach (Human* human, d_humans)
	{
        rgb black;
        black.r = 0; black.g = 0; black.b = 0;
	    // Fill image arrays by default with black
        for(int i=0; i < image_width; i++)
        {
            for(int j=0; j < image_height; j++)
            {
                image_array_filt[j*image_width + i] = black;
                image_array_clust[j*image_width + i] = black;
            }
        }	
	
		// Clear points belonging to humans
		human->points.clear();

		// Determine boundaries
		double x_min = human->x - d_human_depth / 2;
		double x_max = human->x + d_human_depth / 2;
		double y_min = human->y - d_human_width / 2;
		double y_max = human->y + d_human_width / 2;
		double z_min = human->z - d_human_height / 2;
		double z_max = human->z + d_human_height / 2;

		// Use a pass through filter (filter x)
		PCLPointCloudPtr pclCloud_filtered_x (new PCLPointCloud);
		pcl::PassThrough<Point> passthrough_filter_x;
		passthrough_filter_x.setInputCloud(pclCloud_filtered);
		passthrough_filter_x.setFilterFieldName("x");
		passthrough_filter_x.setFilterLimits(x_min, x_max);
		passthrough_filter_x.filter(*pclCloud_filtered_x);

		// Use a pass through filter (filter y)
		PCLPointCloudPtr pclCloud_filtered_y (new PCLPointCloud);
		pcl::PassThrough<Point> passthrough_filter_y;
		passthrough_filter_y.setInputCloud(pclCloud_filtered_x);
		passthrough_filter_y.setFilterFieldName("y");
		passthrough_filter_y.setFilterLimits(y_min, y_max);
		passthrough_filter_y.filter(*pclCloud_filtered_y);

		if (pclCloud_filtered_y->points.size() < 1500)
		{
			ROS_INFO("Not enough points belonging to the human");
			continue;
		}
		
		if(mode == FILTERING || mode == PICTURES)
		{
		    double current_time = ros::Time::now().toSec();
		    
		    for(int i=0; i < pclCloud_filtered_y->points.size(); i++)
		    {
		        pclCloud_segmented_filt->points.push_back(pclCloud_filtered_y->points[i]);
		        //human->points.push_back(pclCloud_filtered_y->points[i]); 
		    }
		  
		    // Store colours into image array
            for(int i=0; i < pclCloud_segmented_filt->points.size(); i++)
            {
                std::stringstream t;
                //ROS_INFO("y_min: %f\t y_max: %f",y_min,y_max);
                // normalised position
                float x_norm = (pclCloud_segmented_filt->points[i].y - y_min) / (y_max - y_min);
                float y_norm = (pclCloud_segmented_filt->points[i].z - z_min) / (z_max - z_min);
                t << "point y: " << pclCloud_segmented_filt->points[i].y;
                //ROS_INFO(t.str().c_str());
                //ROS_INFO("normalised y (x): %f", x_norm);
                
                t.str("");
                t << "x_norm: " << x_norm << " y_norm: "  << y_norm;
                //ROS_INFO(t.str().c_str());
                
                // array position
                int x_target = (int)(x_norm * image_width);
                int y_target = (int)(y_norm * image_height);
                t.str("");
                t << "x_target: " << x_target << " y_target: " << y_target;
                //ROS_INFO(t.str().c_str());
                
                
                t.str("");
                t << "array position: " << y_target*image_width + x_target;
                //ROS_INFO(t.str().c_str());
                if(x_target < image_width && x_target >= 0 && y_target < image_height && y_target >= 0) // to avoid out-of-bounds errors
                {
                    image_array_filt[y_target*image_width + x_target].r = (float)pclCloud_segmented_filt->points[i].r / 255.0;
                    image_array_filt[y_target*image_width + x_target].g = (float)pclCloud_segmented_filt->points[i].g / 255.0;
                    image_array_filt[y_target*image_width + x_target].b = (float)pclCloud_segmented_filt->points[i].b / 255.0;
                }
            }
            ROS_INFO("done writing array");
            
            // Filter out objects next to the main human
            // Human position in array position:
            int human_x = (int)( (human->y - y_min) / (y_max - y_min) * image_width );
            
            float x_max_filter, x_min_filter;   // X-start and X-end of human in meters
            int black_counter = 0;
            // Iterating towards the right
            for(int i=human_x; i < image_width; i++) // width
            {
                bool empty_column = true;
                for(int j=0; j < image_height; j++) // height
                {
                    if(image_array_filt[j*image_width + i].r != 0 || image_array_filt[j*image_width + i].g != 0 || image_array_filt[j*image_width + i].b != 0)
                    {
                        // Array position is not black
                        empty_column = false;
                        break; // No need to look through the rest of the column
                    }                    
                }
                if(empty_column)
                { black_counter++; }
                
                if(black_counter >= gap_limit)
                {
                    //ROS_INFO("i right: %d", i);
                    //ROS_INFO("y_min: %f\t y_max: %f",y_min,y_max);
                    // Space between human and other object is wide enough
                    x_max_filter = ((float)i / image_width) * (y_max - y_min) + y_min;
                    break;
                }
            }
            
            black_counter = 0;
            // Iterating towards the left
            for(int i=human_x; i >= 0; i--) // width
            {
                bool empty_column = true;
                for(int j=0; j < image_height; j++) // height
                {
                    if(image_array_filt[j*image_width + i].r != 0 || image_array_filt[j*image_width + i].g != 0 || image_array_filt[j*image_width + i].b != 0)
                    {
                        // Array position is not black
                        empty_column = false;
                        break; // No need to look through the rest of the column
                    }                    
                }
                if(empty_column)
                { black_counter++; }
                
                if(black_counter >= gap_limit)
                {
                    //ROS_INFO("i left: %d", i);
                    //ROS_INFO("y_min: %f\t y_max: %f",y_min,y_max);
                    // Space between human and other object is wide enough
                    x_min_filter = ((float)i / image_width) * (y_max - y_min) + y_min;
                    break;
                }
            }
            //ROS_INFO("x_min: %f\t x_max: %f",x_min_filter, x_max_filter);
            
		    // Filter the width using the previously determined boundaries
		    PCLPointCloudPtr pclCloud_filtered_objects (new PCLPointCloud);
		    pcl::PassThrough<Point> passthrough_filter_y;
		    passthrough_filter_y.setInputCloud(pclCloud_segmented_filt);
		    passthrough_filter_y.setFilterFieldName("y");
		    passthrough_filter_y.setFilterLimits(x_min_filter, x_max_filter);
		    passthrough_filter_y.filter(*pclCloud_filtered_objects);            
		    
		    pclCloud_segmented_filt->points.clear(); // Reset point cloud
		    for(int i=0; i < pclCloud_filtered_objects->points.size(); i++)
		    {		    
		        pclCloud_segmented_filt->points.push_back(pclCloud_filtered_objects->points[i]);
		        human->points.push_back(pclCloud_filtered_objects->points[i]); 
		    }
		    
		    // Extract features
            getFeatures(pclCloud_filtered_y, human);
		    ROS_INFO("done filtering and finding features");
		    
		    if(frame_iterator < 20)
		    {
		        frame_times[frame_iterator] = ros::Time::now().toSec() - current_time;
		        frame_iterator++;
		    }
		    else
		    {
		        double total_time = 0;
		        for(int i=0; i < 20; i++)
		        {
		            total_time += frame_times[i];
		        }
		        double average_time = total_time / 20.0;
		        ROS_INFO("--> Average filter time: %f", average_time);
		    }
		}
		if(mode == CLUSTERING || mode == PICTURES)
		{
		    double current_time = ros::Time::now().toSec();
		    
		    // We want to only select the biggest cluster
		    // First, extract clusters
		    std::vector<pcl::PointIndices> cluster_indices;     // Cluster information is stored here
		    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);  // Creating the KdTree object for the search method of the extraction
		    tree->setInputCloud (pclCloud_filtered_y);
		    pcl::EuclideanClusterExtraction<Point> ec;
		    ec.setClusterTolerance(0.01);       // was 0.04
		    ec.setMinClusterSize(1000);         // Minimal points that must belong to a cluster // was 1000
		    ec.setMaxClusterSize(2500000);      // Maximal points that must belong to a cluster
		    ec.setSearchMethod(tree);
		    ec.setInputCloud(pclCloud_filtered_y);
		    ec.extract(cluster_indices);

		    // Check if there are any clusters that are big enough
		    if (cluster_indices.size() == 0)
		    {
			    ROS_INFO("Did not find any cluster belonging to the human");
			    continue;
		    }

		    // Determine biggest cluster
		    size_t biggest_cluster = 0;
		    size_t biggest_cluster_size = cluster_indices.front().indices.size();
		    if (cluster_indices.size() > 1)
		    {
			    size_t current_cluster = 1;
			    foreach (pcl::PointIndices indices, cluster_indices)
			    {
				    size_t cluster_size = indices.indices.size();
				    if (cluster_size > biggest_cluster_size)
				    {
					    biggest_cluster_size = cluster_size;
					    biggest_cluster = current_cluster;
				    }
				    ++current_cluster;
			    }
		    }

		    // Copy points of biggest cluster to PointCloud and human
		    pcl::PointIndices point_indices = cluster_indices.at(biggest_cluster);
		    foreach (int index, point_indices.indices)
		    {
			    Point p = pclCloud_filtered_y->points[index];
			    pclCloud_segmented_clust->points.push_back(p);
			    human->points.push_back(p);
		    }
            getFeatures(pclCloud_filtered_y, human);
		    ROS_INFO("done clustering");

		    if(frame_iterator < 20)
		    {
		        frame_times[frame_iterator] = ros::Time::now().toSec() - current_time;
		        frame_iterator++;
		    }
		    else
		    {
		        double total_time = 0;
		        for(int i=0; i < 20; i++)
		        {
		            total_time += frame_times[i];
		        }
		        double average_time = total_time / 20.0;
		        ROS_INFO("--> Average cluster time: %f", average_time);
		    }
		    
	        // Store colours into image array
            for(int i=0; i < pclCloud_segmented_clust->points.size(); i++)
            {
                // normalised position
                float x_norm = (pclCloud_segmented_clust->points[i].y - y_min) / (y_max - y_min);
                float y_norm = (pclCloud_segmented_clust->points[i].z - z_min) / (z_max - z_min);
                
                // array position
                int x_target = (int)(x_norm * image_width);
                int y_target = (int)(y_norm * image_height);
                
                if(x_target < image_width && x_target >= 0 && y_target < image_height && y_target >= 0) // to avoid out-of-bounds errors
                {
                    image_array_clust[y_target*image_width + x_target].r = (float)pclCloud_segmented_clust->points[i].r / 255.0;
                    image_array_clust[y_target*image_width + x_target].g = (float)pclCloud_segmented_clust->points[i].g / 255.0;
                    image_array_clust[y_target*image_width + x_target].b = (float)pclCloud_segmented_clust->points[i].b / 255.0;
                }
            }
            ROS_INFO("done writing array");
		}
		
	    if(mode == PICTURES)
	    {
	        if(image_counter < max_images)
	        {
	            // Save cloud to png file
	            std::stringstream name;
	            name << "human_filt_" << image_counter << ".bmp";
	            saveBMP(name.str().c_str(),image_width,image_height,72,image_array_filt);
	            name.str("");
	            name << "human_clust_" << image_counter << ".bmp";
	            saveBMP(name.str().c_str(),image_width,image_height,72,image_array_clust);
	            
		        image_counter++;
	        }
	        else
	        { ROS_INFO("Gathered enough images."); }
	    }
	}

	// Check which humans to delete
	std::vector<Human*> existing_humans;
	foreach (Human* human, d_humans)
	{
		if (human->points.size() == 0)
		{
			delete human;
		}
		else
		{
			existing_humans.push_back(human);
		}
	}
	d_humans = existing_humans;

	// Publish PointCloud
	pclCloud_segmented_filt->width = pclCloud_segmented_filt->points.size();
	if (d_publish_cloud)
	{
		personCloud_pub.publish(*pclCloud_segmented_filt);
	}
    
    // Write human features to memory
    writeToMemory();

	std::stringstream ss;
	ss << "Humans detected: " << d_humans.size();
	ROS_INFO(ss.str().c_str());
}

void SegmentHumans::checkData()
{
    if(camera_data[0] and camera_data[1])
    {
        *camera_cloud1 += *camera_cloud2;
        
        findHumans(camera_cloud1);
        
        /* Reset the awareness of data so that new data can be received */
        camera_data[0] = false;
        camera_data[1] = false;
    }
}


void SegmentHumans::getFeatures(PCLPointCloudPtr pclCloud_filtered_y, Human* human)
{
    // Get human height
    double height = 0;
    double tmp_height = 0;
    for(int i=0; i < pclCloud_filtered_y->points.size(); i++)
    {
        tmp_height = pclCloud_filtered_y->points[i].z;
        if ( tmp_height > height ) 
        {
            height = tmp_height;
        }            
    }
    human->height = height;
    
    // Use a pass through filter (filter legs)
    PCLPointCloudPtr pclCloud_filtered_legs (new PCLPointCloud);
    pcl::PassThrough<Point> passthrough_filter_legs;
    passthrough_filter_legs.setInputCloud(pclCloud_filtered_y);
    passthrough_filter_legs.setFilterFieldName("z");
    passthrough_filter_legs.setFilterLimits(0.1, 1.0);
    passthrough_filter_legs.filter(*pclCloud_filtered_legs);

    int total_r = 0;
    int total_g = 0;
    int total_b = 0;
    int total_wh = 0;
    int total_gr = 0;
    int total_bl = 0;
    for(int i=0; i < pclCloud_filtered_legs->points.size(); i++)
    {
        int r = pclCloud_filtered_legs->points[i].r;
        int g = pclCloud_filtered_legs->points[i].g;
        int b = pclCloud_filtered_legs->points[i].b;
        if ( r > g && r > b && ( r - g > 30 )  && ( r - b > 30 )) 
        {
            total_r++;
        }
        else if ( g > r && g > b && ( g - r > 30 )  && ( g - b > 30 )) 
        {
            total_g++;
        }
        else if ( b > r && b > g && ( b - r > 30 )  && ( b - g > 30 )) 
        {
            total_b++;
        }
        else if ( ( r + b + g ) < 150 ) 
        {
            total_bl++;
        }
        else if ( ( r + b + g ) >= 750 )
        {
            total_wh++;
        }
        else 
        {
            total_gr++;
        }
    }
    int n_points;
    if(pclCloud_filtered_legs->points.size() == 0)
    { n_points = 1; }
    else
    { n_points = pclCloud_filtered_legs->points.size(); }
    human->legs_red = (double)total_r / n_points;
    human->legs_green = (double)total_g / n_points;
    human->legs_blue = (double)total_b / n_points;
    human->legs_black = (double)total_bl / n_points;
    human->legs_white = (double)total_wh / n_points;
    human->legs_grey = (double)total_gr / n_points;
    
    // Use a pass through filter (filter upper body)
    PCLPointCloudPtr pclCloud_filtered_upper (new PCLPointCloud);
    pcl::PassThrough<Point> passthrough_filter_upper;
    passthrough_filter_upper.setInputCloud(pclCloud_filtered_y);
    passthrough_filter_upper.setFilterFieldName("z");
    passthrough_filter_upper.setFilterLimits(1.0, 2.2);
    passthrough_filter_upper.filter(*pclCloud_filtered_upper);

    total_r = 0;
    total_g = 0;
    total_b = 0;
    total_wh = 0;
    total_gr = 0;
    total_bl = 0;
    for(int i=0; i < pclCloud_filtered_upper->points.size(); i++)
    {
        int r = pclCloud_filtered_upper->points[i].r;
        int g = pclCloud_filtered_upper->points[i].g;
        int b = pclCloud_filtered_upper->points[i].b;
        if ( r > g && r > b && ( r - g > 30 )  && ( r - b > 30 )) 
        {
            total_r++;
        }
        else if ( g > r && g > b && ( g - r > 30 )  && ( g - b > 30 )) 
        {
            total_g++;
        }
        else if ( b > r && b > g && ( b - r > 30 )  && ( b - g > 30 )) 
        {
            total_b++;
        }
        else if ( ( r + b + g ) < 150 ) 
        {
            total_bl++;
        }
        else if ( ( r + b + g ) >= 750 )
        {
            total_wh++;
        }
        else 
        {
            total_gr++;
        }
    }
    human->upper_red = (double)total_r / pclCloud_filtered_upper->points.size();
    human->upper_green = (double)total_g / pclCloud_filtered_upper->points.size();
    human->upper_blue = (double)total_b / pclCloud_filtered_upper->points.size();
    human->upper_black = (double)total_bl / pclCloud_filtered_upper->points.size();
    human->upper_white = (double)total_wh / pclCloud_filtered_upper->points.size();
    human->upper_grey = (double)total_gr / pclCloud_filtered_upper->points.size();
}

void SegmentHumans::saveBMP(const char *filename, int w, int h, int dpi, rgb *data)
{
	FILE *f;
	int k = w*h;
	int s = 4*k;
	int filesize = 54 + s;
	
	double factor = 39.375;
	int m = static_cast<int>(factor);
	
	int ppm = dpi*m;
	
	unsigned char bmpfileheader[14] = {'B','M', 0,0,0,0, 0,0,0,0, 54,0,0,0};
	unsigned char bmpinfoheader[40] = {40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0,24,0};
	
	bmpfileheader[ 2] = (unsigned char)(filesize);
	bmpfileheader[ 3] = (unsigned char)(filesize>>8);
	bmpfileheader[ 4] = (unsigned char)(filesize>>16);
	bmpfileheader[ 5] = (unsigned char)(filesize>>24);
	
	bmpinfoheader[ 4] = (unsigned char)(w);
	bmpinfoheader[ 5] = (unsigned char)(w>>8);
	bmpinfoheader[ 6] = (unsigned char)(w>>16);
	bmpinfoheader[ 7] = (unsigned char)(w>>24);
	
	bmpinfoheader[ 8] = (unsigned char)(h);
	bmpinfoheader[ 9] = (unsigned char)(h>>8);
	bmpinfoheader[10] = (unsigned char)(h>>16);
	bmpinfoheader[11] = (unsigned char)(h>>24);
	
	bmpinfoheader[21] = (unsigned char)(s);
	bmpinfoheader[22] = (unsigned char)(s>>8);
	bmpinfoheader[23] = (unsigned char)(s>>16);
	bmpinfoheader[24] = (unsigned char)(s>>24);
	
	bmpinfoheader[25] = (unsigned char)(ppm);
	bmpinfoheader[26] = (unsigned char)(ppm>>8);
	bmpinfoheader[27] = (unsigned char)(ppm>>16);
	bmpinfoheader[28] = (unsigned char)(ppm>>24);
	
	bmpinfoheader[29] = (unsigned char)(ppm);
	bmpinfoheader[30] = (unsigned char)(ppm>>8);
	bmpinfoheader[31] = (unsigned char)(ppm>>16);
	bmpinfoheader[32] = (unsigned char)(ppm>>24);
	
	f = fopen(filename,"wb");
	
	fwrite(bmpfileheader,1,14,f);
	fwrite(bmpinfoheader,1,40,f);
	
	for (int i = 0; i < k; i++) {
		//RGBType rgb = data[i];
		
		double red = (data[i].r)*255;
		double green = (data[i].g)*255;
		double blue = (data[i].b)*255;
		
		unsigned char color[3] = {(int)floor(blue),(int)floor(green),(int)floor(red)};
		
		fwrite(color,1,3,f);
	}
	
	fclose(f);
}

int main(int argc, char** argv)
{
    // ROS initialization stuff
	ros::init(argc, argv, "segment_humans");
	ros::NodeHandle n;
	ros::Rate r(10);

	// Create listener and set global pointer
	tf::TransformListener temp_listener;
	listener = &temp_listener;

	// Create memory service client
	memory_client = n.serviceClient<borg_pioneer::MemoryReadSrv>("memory_read");
	memory_client_write = n.serviceClient<borg_pioneer::MemorySrv>("memory");

	// Get parameters
	bool publish_cloud;			ros::param::param<bool>("~publish_cloud", publish_cloud, true);
	std::string camera_topic3;	ros::param::param<std::string>("~camera_topic3", camera_topic3, "camera3/depth_registered/points");
	std::string camera_topic2;  ros::param::param<std::string>("~camera_topic2", camera_topic2, "camera2/depth_registered/points");

	// Create segmentizer/tracker
	SegmentHumans sh(publish_cloud);
	sh.initImageArrays();

	//Subscribe to point cloud data
	ros::Subscriber depth_sub = n.subscribe<sensor_msgs::PointCloud2>(camera_topic3, 1, &SegmentHumans::processCam1, &sh);
	ros::Subscriber depth_sub2 = n.subscribe<sensor_msgs::PointCloud2>(camera_topic2, 1, &SegmentHumans::processCam2, &sh);

	// Create a ROS publisher for the output PointCloud
	personCloud_pub = n.advertise<PCLPointCloud>("human_cloud/output", 1);
	
	// Create window for color histograms
	cv::namedWindow("Color Histograms", cv::WINDOW_NORMAL);

	ros::spin();

    return 0;
}
