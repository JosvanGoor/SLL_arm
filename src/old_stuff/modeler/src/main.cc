#include <modeler/utils.h>
#include <modeler/recorder.h>

/*
#include <iostream>
#include <vector> 
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>

#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/shot_omp.h>

#include <pcl/recognition/ransac_based/obj_rec_ransac.h>
#include <pcl/recognition/cg/geometric_consistency.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>

#include <pcl_ros/segmentation/extract_clusters.h>
#include <pcl/registration/ndt.h>


// boost includes
#include <boost/algorithm/string/predicate.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>


using namespace std;
using namespace sensor_msgs;
using namespace cv;
using namespace pcl;

typedef PointXYZRGB PCLPoint;
typedef pcl::PointCloud<PCLPoint> PCLPointCloud;
typedef pcl::PointCloud<PCLPoint>::Ptr PCLPointCloudPtr;

using boost::property_tree::ptree;

#ifdef __CDT_PARSER__
    #define foreach(a, b) for(a : b)
#else
    #define foreach(a, b) BOOST_FOREACH(a, b)
#endif


ros::Publisher cloud_pub;

Mat k;

ros::Subscriber cam_info_sub;

bool cameraInfoReady = false;
float prevRotation = 9000.1f;
float uni_sample_size = 0.015f;

PCLPointCloudPtr object_cloud(new PCLPointCloud);

vector<cv::Point3f>  calcChessboardCorners(Size boardSize, float squareSize, Point3f offset = Point3f())
{
	vector<Point3f> corners;

	for (int i = 0; i < boardSize.height; i++)
		for (int j = 0; j < boardSize.width; j++)
			corners.push_back(Point3f(float(i * squareSize), float((2 * j + i % 2) * squareSize), 0) + offset);
	
	return corners;
}

void draw(cv::Mat& drawImage, const cv::Mat& K, const cv::Mat& R, const cv::Mat T)
{
	if (R.empty() || T.empty())
		return;
	float scale = 0.1;
	Point3f z(0, 0, scale);
	Point3f x(scale, 0, 0);
	Point3f y(0, scale, 0);
	Point3f o(0, 0, 0);
	vector<Point3f> op(4);
	op[1] = x, op[2] = y, op[3] = z, op[0] = o;
	vector<Point2f> ip;
	projectPoints(Mat(op), R, T, K, Mat(4, 1, CV_64FC1, Scalar(0)), ip);

	vector<Scalar> c(4); //colors
	c[0] = Scalar(255, 255, 255);
	c[1] = Scalar(255, 0, 0); //x
	c[2] = Scalar(0, 255, 0); //y
	c[3] = Scalar(0, 0, 255); //z
	line(drawImage, ip[0], ip[1], c[1]);
	line(drawImage, ip[0], ip[2], c[2]);
	line(drawImage, ip[0], ip[3], c[3]);
	string scaleText = boost::str(boost::format("scale %0.2f meters")%scale);
	int baseline = 0;
	Size sz = getTextSize(scaleText, CV_FONT_HERSHEY_SIMPLEX, 1, 1, &baseline);
	Point box_origin(10, drawImage.size().height - 10);
	rectangle(drawImage, box_origin + Point(0, 5), box_origin + Point(sz.width, -sz.height - 5), Scalar::all(0), -1);
	putText(drawImage, scaleText, box_origin, CV_FONT_HERSHEY_SIMPLEX, 1.0, c[0], 1, CV_AA, false);
	putText(drawImage, "Z", ip[3], CV_FONT_HERSHEY_SIMPLEX, 1.0, c[3], 1, CV_AA, false);
	putText(drawImage, "Y", ip[2], CV_FONT_HERSHEY_SIMPLEX, 1.0, c[2], 1, CV_AA, false);
	putText(drawImage, "X", ip[1], CV_FONT_HERSHEY_SIMPLEX, 1.0, c[1], 1, CV_AA, false);

}


Eigen::Matrix4f getTransformation(Mat image)
{
    Mat imageGray;
    cvtColor(image, imageGray, CV_RGB2GRAY);    

    vector<Point2f> out_bw;
    vector<Point2f> out_wb;

    Size grid_size = Size(3, 5); // pattern size 

    // find black dots, white background pattern
    bool patternFoundBw = findCirclesGrid(imageGray, grid_size, out_bw, CALIB_CB_ASYMMETRIC_GRID | CALIB_CB_CLUSTERING);

    bitwise_not(imageGray, imageGray);  // reverse image colors (turn black to white, and white to black)
    
    // find white dots, black background pattern
    bool patternFoundWb = findCirclesGrid(imageGray, grid_size, out_wb, CALIB_CB_ASYMMETRIC_GRID | CALIB_CB_CLUSTERING);
    
    drawChessboardCorners(image, grid_size, out_bw, patternFoundBw);
    drawChessboardCorners(image, grid_size, out_wb, patternFoundWb);
   
    float square_size = 0.02f;
    Point3f offset; // pattern offsets
	offset.z = 0.00f; 
	offset.y = -0.05f;

	vector<Point3f> ideal_pts;
	vector<Point2f> out;
	Mat rvec, tvec;

    if (patternFoundBw)
    {
	    offset.x = -0.157;    
	    
	    vector<Point3f> ideal_pts_bw = calcChessboardCorners(grid_size, square_size, offset);
	    ideal_pts.insert(ideal_pts.end(), ideal_pts_bw.begin(), ideal_pts_bw.end());
	    out.insert(out.end(), out_bw.begin(), out_bw.end());

	//    solvePnP(ideal_pts_bw, out_bw, k, Mat(), rvec, tvec, false);  
    //	draw(image, k, rvec, tvec);

	}

	if (patternFoundWb)
	{
    	offset.x = 0.075;

    	vector<Point3f> ideal_pts_wb = calcChessboardCorners(grid_size, square_size, offset);
    	ideal_pts.insert(ideal_pts.end(), ideal_pts_wb.begin(), ideal_pts_wb.end());
    	out.insert(out.end(), out_wb.begin(), out_wb.end());   

    //	solvePnP(ideal_pts_wb, out_wb, k, Mat(), rvec, tvec, false);  
    //	draw(image, k, rvec, tvec); 	
    }

    if (patternFoundBw || patternFoundWb)  // a pattern was found, so we have translation and rotation
    {
	///	Mat rvec, tvec;	

    	solvePnP(ideal_pts, out, k, Mat(), rvec, tvec, false);  
    	draw(image, k, rvec, tvec);

        Mat rmat;
        Rodrigues(rvec, rmat); // transform rotation vector to rotation matrix
    
    //    ROS_INFO_STREAM("ROTATION VEC: " << rvec);
    //    ROS_INFO_STREAM("ROT MATRIX:\n " << rmat);

   //     ROS_INFO_STREAM("TRANSLATION: \n" << tvec);

        Eigen::Matrix4f Tm = Eigen::Matrix4f::Zero();
        Tm(3,3) = 1;
        for (size_t idx = 0; idx < 3; ++idx)
            for (size_t idy = 0; idy < 3; ++idy)
                Tm (idx,idy) = rmat.at<double>(idx,idy);

        Tm(0,3) = (float)tvec.at<double>(0);
        Tm(1,3) = (float)tvec.at<double>(1);
        Tm(2,3) = (float)tvec.at<double>(2);

    //    ROS_INFO_STREAM("NEW ROT MATRIX:\n " << Tm);

        imshow("image", image);
        waitKey(1);
        return Tm;
    }
    else
    {
        imshow("image", image);
        waitKey(1);
        return Eigen::Matrix4f::Zero();
    }

    

}

Mat makeImage(PCLPointCloud cloud)
{
    int width = cloud.height;
    int height = cloud.width;
    cv::Mat newImage(width, height, CV_8UC3);
    
    int count = 0;
        
    for (size_t idx = 0; idx < width; ++idx)
    {
        for (size_t idy = 0; idy < height; ++idy)
        {
            int b = cloud.points[count].b;
            int g = cloud.points[count].g;
            int r = cloud.points[count].r;
            newImage.at<cv::Vec3b>(idx, idy)[0] = (unsigned char)b;
            newImage.at<cv::Vec3b>(idx, idy)[1] = (unsigned char)g;
            newImage.at<cv::Vec3b>(idx, idy)[2] = (unsigned char)r;                 
            ++count;    
        }
    }

    return newImage;
}

bool firstPrint = true;

void getKeyPoints(PCLPointCloud &incloud)
{
	PCLPointCloudPtr cloud(new PCLPointCloud);
	*cloud = incloud;
	pcl::PointCloud<int> sampled_indices;
    UniformSampling<PCLPoint> detector;
    detector.setRadiusSearch(uni_sample_size);
    detector.setInputCloud(cloud);    
    detector.compute(sampled_indices);
    
    PCLPointCloudPtr cloud_keypoints (new PCLPointCloud);
    pcl::copyPointCloud (*cloud, sampled_indices.points, *cloud_keypoints);

    for (size_t idx = 0; idx < cloud_keypoints->size(); ++idx)
    {
    	if (firstPrint)
    	{
    		cloud_keypoints->points[idx].r = 255;
    		cloud_keypoints->points[idx].g = 0;
    		cloud_keypoints->points[idx].b = 0;
    		
    	}
    	else
    	{
    		cloud_keypoints->points[idx].r = 0;
    		cloud_keypoints->points[idx].g = 0;
    		cloud_keypoints->points[idx].b = 255;
    	}

    }
    firstPrint = false;
    *object_cloud += *cloud_keypoints;
    cloud_pub.publish(*object_cloud);
}

bool bTransform = false;
bool multiFound = false;

void pcCallBack(const PCLPointCloud::ConstPtr &cloudMsg)
{
    Mat image = makeImage(*cloudMsg);
    Eigen::Matrix4f transformMatrix = getTransformation(image);

    if (transformMatrix == Eigen::Matrix4f::Zero())
        return;

//    ROS_INFO_STREAM("Z-Rot: " << atan2(transformMatrix(0,1),transformMatrix(0,0)) * 180 / M_PI);

    float currentRotation = atan2(transformMatrix(0,1),transformMatrix(0,0)) * 180 / M_PI;
    
    if (multiFound || (prevRotation != 9000.1f && abs(prevRotation - currentRotation) >= 1.0f))
    {
    	prevRotation = currentRotation;
    	PCLPointCloudPtr cloud (new PCLPointCloud);

	    cloud->header.frame_id = "camera_rgb_optical_frame";

	//    ROS_INFO_STREAM("TRANSLATION\n:" << transformMatrix(0,3) << ",  " << transformMatrix(1,3) << ", " << transformMatrix(2,3));

	 //   float zLow = transformMatrix(2,3);
	//    float zHigh = zLow + 0.3f;

	    Eigen::Matrix4f invMatrix = transformMatrix.inverse();
	    pcl::transformPointCloud (*cloudMsg, *cloud, invMatrix);
	//    *cloud = *cloudMsg;

	    PassThrough<PCLPoint> passthrough_filter;
		passthrough_filter.setInputCloud(cloud);
		passthrough_filter.setFilterFieldName("z");
		passthrough_filter.setFilterLimits(0, transformMatrix(2,3) - 0.00f);
	//	passthrough_filter.setFilterLimits(0.4, 0.70f);
		passthrough_filter.filter(*cloud);	

		// filter Y		
		passthrough_filter.setInputCloud(cloud);
		passthrough_filter.setFilterFieldName("y");
	//	passthrough_filter.setFilterLimits(-0.2 , 0.2;
		passthrough_filter.setFilterLimits(transformMatrix(1,3) - 0.2, transformMatrix(1,3) + 0.2);
		passthrough_filter.filter(*cloud);

		// filter X 
		passthrough_filter.setInputCloud(cloud);
		passthrough_filter.setFilterFieldName("x");
	//	passthrough_filter.setFilterLimits(-0.20, 0.20);
		passthrough_filter.setFilterLimits(transformMatrix(0,3) - 0.20, transformMatrix(0,3) + 0.20);
		passthrough_filter.filter(*cloud);

		ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		PointIndices::Ptr inliers (new pcl::PointIndices);
		// Create the segmentation object
		SACSegmentation<PCLPoint> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold (0.02);  // 0.015 

		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);		
		
		ExtractIndices<PCLPoint> extract;

		extract.setInputCloud(cloud);
	    extract.setIndices(inliers);
	    extract.setNegative(true); // true means remove planes
	    extract.filter(*cloud);


	  //  pcl::transformPointCloud (*cloud, *cloud, transformMatrix);

	    // for testing to see what is being used for clustering
	    cloud_pub.publish(cloud);
	    return;

	    vector<pcl::PointIndices> cluster; 
		pcl::search::KdTree<PCLPoint>::Ptr tree (new pcl::search::KdTree<PCLPoint>);
    	tree->setInputCloud(cloud);
		EuclideanClusterExtraction<PCLPoint> ec;	
	
		ec.setClusterTolerance(0.01);       // in meters
		ec.setMinClusterSize(100);       // Minimal points that must belong to a cluster
		ec.setMaxClusterSize(40000);      // Maximal points that must belong to a cluster
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud);
		ec.extract(cluster);

		if (cluster.size() > 1)
		{
			ROS_INFO_STREAM("Found more then one cluster!!!!!!!!!");
			multiFound = true;
			return;
		}

		multiFound = false;
	//	return;

		for (size_t idc = 0; idc < cluster.size(); ++idc)
		{	
			PCLPointCloudPtr objectCloud1(new PCLPointCloud);
			PCLPointCloudPtr newCloud(new PCLPointCloud);
			pcl::PointIndices point_indices = cluster.at(idc);

			foreach (int index, point_indices.indices)			
			{
				PCLPoint p = cloud->points[index];				
				objectCloud1->points.push_back(p);
			}
			 
	 //   	pcl::transformPointCloud (*objectCloud1, *objectCloud1, invMatrix);
			PCLPointCloudPtr objectCloud2(new PCLPointCloud);
	    	
	    	if (bTransform)
	    	{
	    	/*	Eigen::Matrix4f tfTest = Eigen::Matrix4f::Identity();
	    		tfTest(0,3) = -0.05f;
	    		tfTest(1,3) = -0.00f;
	    		tfTest(2,3) = -0.00f;
	    		
	    		pcl::transformPointCloud (*objectCloud1, *objectCloud2, tfTest);
	    		pcl::transformPointCloud (*objectCloud1, *objectCloud1, tfTest);

	    		ROS_INFO_STREAM("SIZE: " << objectCloud2->size());

	    		for (size_t idx = 0; idx < objectCloud2->size(); ++idx)
			    {
		       		objectCloud2->points[idx].r = 255;
		    		objectCloud2->points[idx].g = 0;
		    		objectCloud2->points[idx].b = 0;			    		
			    }
		
			    
	    		PCLPointCloudPtr output_cloud(new PCLPointCloud);
	    		pcl::NormalDistributionsTransform<PCLPoint, PCLPoint> ndt;
	    		ndt.setTransformationEpsilon(0.00001);
	    		ndt.setStepSize (0.1);
	    		ndt.setResolution (0.005);
	    		ndt.setMaximumIterations (500);
	    		ndt.setInputSource (objectCloud1);
	    		ndt.setInputTarget (object_cloud);
	    		ndt.align (*output_cloud);

	    		std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
            		<< " score: " << ndt.getFitnessScore () << std::endl;

            	Eigen::Matrix4f ftransform = ndt.getFinalTransformation();
            	Eigen::Matrix4f inv_ftransform = ftransform.inverse(); 
				pcl::transformPointCloud (*objectCloud1, *newCloud, ftransform);
				
	    	}

	    	if (!bTransform)
	   			*object_cloud += *objectCloud1;
	   		else
	   		{
	   			*object_cloud += *newCloud;
	  // 			*object_cloud += *objectCloud2;
	   		}
	   		
	    	bTransform = true;			
		}

	    object_cloud->header.frame_id = "camera_rgb_optical_frame"; 
	    
	    ROS_INFO_STREAM("OUT SIZE: "  << object_cloud->size());
	    cloud_pub.publish(*object_cloud); 
    }
    else if (prevRotation == 9000.1f)
    	prevRotation = currentRotation;


    

}

void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr cam_info)
{
	Mat D;
	CameraInfo ci = *cam_info;
	k = cv::Mat(3, 3, CV_32FC1);
	
	for (int i = 0; i < 9; i++)
		k.at<float>(i / 3, i % 3) = ci.K[i];
	
	D = cv::Mat(ci.D.size(), 1, CV_32FC1);
		
	for (size_t i = 0; i < ci.D.size(); i++)
		D.at<float>(i) = ci.D[i];

	cameraInfoReady = true;
	cam_info_sub.shutdown();
}


*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "objectDetection");
	ros::NodeHandle nh;

	Recorder recorder;

	//Subscribe to point cloud data
//	ros::Subscriber rgb_sub = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 1, rgbCallback); // for the newest xtion
//    ros::Subscriber rgb_sub = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_color", 1, rgbCallback);

 //   ros::Subscriber depth_sub = nh.subscribe<PCLPointCloud>("/camera/depth_registered/points", 1, pcCallBack);

//	cam_info_sub = nh.subscribe<sensor_msgs::CameraInfo>("/camera/rgb/camera_info", 1, camInfoCallback);

//    cloud_pub = nh.advertise<PCLPointCloud>("transformation/model", 1);

	ros::spin();

}