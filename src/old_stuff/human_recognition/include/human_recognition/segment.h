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
//#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/filters/voxel_grid.h>
//#include <pcl/filters/passthrough.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

// Clustering
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using boost::property_tree::ptree;

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PCLPointCloud;
typedef pcl::PointCloud<Point>::Ptr PCLPointCloudPtr;

const float THRESHOLD =     0.0001;

const int FILTERING =       0;
const int CLUSTERING =      1;
const int PICTURES =        2;  // Both filtering and clustering

// Colours
const int RED =             0;
const int BLUE =            1;
const int GREEN =           2;
const int YELLOW =          3;
const int BLACK =           4;
const int WHITE =           5;
const int GREY =            6;

// HSV Color Divide Thresholds
const float V_BLACK =		    0.1;
const float V_WHITE = 		    0.8;
const float S_GREY =		    0.2;
const int H_RED_LOW_CENTER =    0;
const int H_RED_YELLOW =        25;     // was 40
const int H_YELLOW_CENTER =     60;
const int H_YELLOW_GREEN =      70;     // was 80
const int H_GREEN_CENTER =      120;
const int H_GREEN_BLUE =        160;    // was 180
const int H_BLUE_CENTER =       240;
const int H_BLUE_RED =          300;    // was 300
const int H_RED_HIGH_CENTER =   360;

// Body
const int UPPER =           0;
const int LEGS =            1;

#ifdef __CDT_PARSER__
    #define foreach(a, b) for(a : b)
#else
    #define foreach(a, b) BOOST_FOREACH(a, b)
#endif

struct rgb
{
    float r;
    float g;
    float b;
};

struct Human
{
	float x;
	float y;
	float z;
	float x_pix;
	float y_pix;
	std::vector<Point> points;
    
    // Features
    rgb max_channels;
    rgb min_channels;
    double height;
    float legs_red;
    float legs_green;
    float legs_blue;
    float legs_yellow;
    float legs_white;
    float legs_black;
    float legs_grey;
    float upper_red;
    float upper_green;
    float upper_blue;
    float upper_yellow;
    float upper_white;
    float upper_black;
    float upper_grey;
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
	double leg_height;              // Approx height of a human's legs
    float voxel_size;               // Reduction size for the voxelgrid filter in m
	bool d_publish_cloud;			// Publish segmented humans (Y/N)
	bool display_histogram;         // If color histograms should be drawn
	int mode;                       // What mode of data analysis and output should be used
	int image_counter;
	int max_images;
	int gap_limit;                  // Size of allowed gap in array between person and object
	float image_ratio;              // Amount the image should be scaled by from cloud dimensions
	int image_width;                // The width of the image output
	int image_height;               // The height of the image output
	rgb *image_array_filt;          // The array that holds the colours of the image for the filtering method
	rgb *image_array_clust;         // The array that holds the colours of the image for the clustering method
	double frame_time;              // The time used to record the length of frames
    int n_frames;                   // Number of frames to record for FPS calculations
	int frame_iterator;
    
    // Debug
    std::vector<float> h_values;

//Functions
public:
    SegmentHumans(bool publish_cloud);
    ~SegmentHumans();
    void initImageArrays();
	void readHumanDetectionsFromMemory(std::string message);
	void readFromMemory();
    void writeToMemory();
	void depthCallback(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);
	bool availabilityChecks(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);
	void processCam(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);
	PCLPointCloudPtr getCloud(const sensor_msgs::PointCloud2ConstPtr &cloudMsg);
	void findHumans(PCLPointCloudPtr pclCloud);
    void drawHistograms(bool display);
    void getFeatures(PCLPointCloudPtr pclCloud, Human* human);
    float getColourData(Human *human, const int portion, const int colour);
    float computeColourPortion(float h, const int center, const int lower_bound, const int upper_bound);
	void saveBMP(const char *filename, int w, int h, int dpi, rgb *data);
	
	float max3(float a, float b, float c);
	float min3(float a, float b, float c);

	float divideColors(float h, float s, float v);
};
