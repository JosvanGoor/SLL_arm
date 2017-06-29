#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>

#include <pcl/common/common_headers.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/io/pcd_io.h>


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>

#include <vector>
#include <sensor_msgs/CameraInfo.h>
#include <sstream>


using namespace pcl;
using namespace std;
using namespace ros;
using namespace cv;
using namespace sensor_msgs;

typedef PointXYZRGB PCLPoint;
typedef pcl::PointCloud<PCLPoint> PCLPointCloud;
typedef pcl::PointCloud<PCLPoint>::Ptr PCLPointCloudPtr;

using boost::property_tree::ptree;

#ifdef __CDT_PARSER__
    #define foreach(a, b) for(a : b)
#else
    #define foreach(a, b) BOOST_FOREACH(a, b)
#endif


// **** VARIABLES ****
int bottomSlider;
int bottomSliderMax;
float bottomCutoff;
int zeroPointSlider;
int zeroPointSliderMax;
float zeroPointCutoff;

int leftSlider, leftSliderMax, rightSlider, rightSliderMax;
float leftSliderCutoff, rightSliderCutoff;

int frontSlider, frontSliderMax, backSlider, backSliderMax;
float frontSliderCutoff, backSliderCutoff;

Mat d_mK;
Mat imageObject;
int count2;
string dirname;

ros::Publisher cloudPub;
ros::Subscriber d_cam_info_sub;

PCLPointCloudPtr cloud(new PCLPointCloud);


// Function declarations
Mat makeImage(PCLPointCloud cloud);
Eigen::Matrix4f getTransformation(Mat image);
vector<cv::Point3f>  calcChessboardCorners(Size boardSize, float squareSize, Point3f offset);
void draw(cv::Mat& drawImage, const cv::Mat& K, const cv::Mat& R, const cv::Mat T);


// Functions

void create2DImage(Mat &image, PCLPointCloud &unmodCloud, PCLPointCloud &cloud)
{
//	int width = unmodCloud.height;
//	int height = unmodCloud.width;
//	cv::Mat newImage(width, height, CV_8UC3);

	cv::Mat backupImage;
	backupImage = image.clone();
	int count1 = 0;

	size_t left = image.rows;
	size_t right = 0;
	size_t bottom = image.cols;
	size_t top = 0;

	for (size_t idx = 0; idx < image.rows; ++idx)
		for (size_t idy = 0; idy < image.cols; ++idy)
		{
			if ((unmodCloud.points[count1].z < zeroPointCutoff + bottomCutoff ||
					((unmodCloud.points[count1].x > leftSliderCutoff || unmodCloud.points[count1].x < -rightSliderCutoff) ||
					(unmodCloud.points[count1].y < -backSliderCutoff || unmodCloud.points[count1].y > frontSliderCutoff))))
			{
				image.at<cv::Vec3b>(idx, idy)[0] = (unsigned char)0;
				image.at<cv::Vec3b>(idx, idy)[1] = (unsigned char)0;
				image.at<cv::Vec3b>(idx, idy)[2] = (unsigned char)0;
			}
			else // find left, right, top, and bottom index
			{
				if (isFinite(unmodCloud.points[count1]))
				{
					if (idy > top)
						top = idy;
					if (idy < bottom)
						bottom = idy;
					if (idx < left)
						left = idx;
					if (idx > right)
						right = idx;

				}
			}

			++count1;
		}

	count1 = 0;

	for (size_t idx = 0; idx < image.rows; ++idx)
			for (size_t idy = 0; idy < image.cols; ++idy)
			{
				if (!isFinite(unmodCloud.points[count1]))
				{
					if (idx >= left && idx <= right &&
							idy >= bottom && idy <= top)
					{
						image.at<cv::Vec3b>(idx, idy)[0] = backupImage.at<cv::Vec3b>(idx, idy)[0];
						image.at<cv::Vec3b>(idx, idy)[1] = backupImage.at<cv::Vec3b>(idx, idy)[1];
						image.at<cv::Vec3b>(idx, idy)[2] = backupImage.at<cv::Vec3b>(idx, idy)[2];
					}
					else
					{
						image.at<cv::Vec3b>(idx, idy)[0] = (unsigned char)0;
						image.at<cv::Vec3b>(idx, idy)[1] = (unsigned char)0;
						image.at<cv::Vec3b>(idx, idy)[2] = (unsigned char)0;
					}
				}

				++count1;

			}

	imageObject = image;
	cv::imshow("object2d", image);
}

void pcCallBack(const PCLPointCloud::ConstPtr &cloudMsg)
{
	Mat image = makeImage(*cloudMsg);
	Mat backupImage = image.clone();
	Eigen::Matrix4f transformMatrix = getTransformation(image); // find the transformation from the pattern

	if (transformMatrix == Eigen::Matrix4f::Zero()) // if patterns are not found, we don't know the transformation
		return;

	PCLPointCloudPtr unmodCloud(new PCLPointCloud);
	cloud->header.frame_id = "xtion_rgb_optical_frame"; // needed when publishing the cloud
	Eigen::Matrix4f invMatrix = transformMatrix.inverse();
	pcl::transformPointCloud (*cloudMsg, *unmodCloud, invMatrix);

	PassThrough<PCLPoint> passthrough_filter;
	passthrough_filter.setInputCloud(unmodCloud);
	passthrough_filter.setFilterFieldName("z");
	passthrough_filter.setFilterLimits(zeroPointCutoff + bottomCutoff, 1.5);
	passthrough_filter.filter(*cloud);

	passthrough_filter.setInputCloud(cloud);
	passthrough_filter.setFilterFieldName("y");
	passthrough_filter.setFilterLimits(-backSliderCutoff, frontSliderCutoff);
	passthrough_filter.filter(*cloud);

	passthrough_filter.setInputCloud(cloud);
	passthrough_filter.setFilterFieldName("x");
	passthrough_filter.setFilterLimits(-rightSliderCutoff, leftSliderCutoff);
	passthrough_filter.filter(*cloud);

	// Not working correctly, needs more manual control
	create2DImage(backupImage, *unmodCloud, *cloud);
	cloudPub.publish(cloud);

	cv::waitKey(1);
}

void onFrontSliderChange(int, void*)
{
	frontSliderCutoff = (float) frontSlider / 1000;
}

void onBackSliderChange(int, void*)
{
	backSliderCutoff = (float) backSlider / 1000;
}

void onBottomSliderChange(int, void*)
{
	bottomCutoff = (float)bottomSlider / 1000;
}

void onLeftSliderChange(int, void*)
{
	leftSliderCutoff = (float)leftSlider / 1000;
}

void onRightSliderChange(int, void*)
{
	rightSliderCutoff = (float)rightSlider / 1000;
}

void onZeroPointSliderChange(int, void*)
{
	zeroPointCutoff = (float)zeroPointSlider / 100 - 0.7f;
}

vector<cv::Point3f>  calcChessboardCorners(Size boardSize, float squareSize, Point3f offset)
{
    vector<Point3f> corners;

    for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
            corners.push_back(Point3f(float(i * squareSize), float((2 * j + i % 2) * squareSize), 0) + offset);
    	//	corners.push_back(Point3f(float((2*j + i % 2)*squareSize),float(i*squareSize), 0));

    return corners;
}

void draw(cv::Mat& drawImage, const cv::Mat& K, const cv::Mat& R, const cv::Mat T)
{
    if (R.empty() || T.empty())
        return;
    float scale = 0.065;
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
  //  rectangle(drawImage, box_origin + Point(0, 5), box_origin + Point(sz.width, -sz.height - 5), Scalar::all(0), -1);
 //   putText(drawImage, scaleText, box_origin, CV_FONT_HERSHEY_SIMPLEX, 1.0, c[0], 1, CV_AA, false);
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

    float square_size = 0.02f; // 0.02f
    Point3f offset; // pattern offsets
    offset.z = 0.00f;
    offset.y = -0.05f;  // -0.05f;

    vector<Point3f> ideal_pts;
    vector<Point2f> out;
    Mat rvec, tvec;

    if (patternFoundBw)
    {
        offset.x = -0.157f;   // -0.157

        vector<Point3f> ideal_pts_bw = calcChessboardCorners(grid_size, square_size, offset);
        ideal_pts.insert(ideal_pts.end(), ideal_pts_bw.begin(), ideal_pts_bw.end());
        out.insert(out.end(), out_bw.begin(), out_bw.end());
    }

    if (patternFoundWb)
    {
        offset.x = 0.075f; // 0.075;

        vector<Point3f> ideal_pts_wb = calcChessboardCorners(grid_size, square_size, offset);
        ideal_pts.insert(ideal_pts.end(), ideal_pts_wb.begin(), ideal_pts_wb.end());
        out.insert(out.end(), out_wb.begin(), out_wb.end());
    }

    if (patternFoundBw || patternFoundWb)  // a pattern was found, so we have translation and rotation
    {
        solvePnP(ideal_pts, out, d_mK, Mat(), rvec, tvec, false);
        draw(image, d_mK, rvec, tvec);

        Mat rmat;
        Rodrigues(rvec, rmat); // transform rotation vector to rotation matrix

        Eigen::Matrix4f Tm = Eigen::Matrix4f::Zero();
        Tm(3,3) = 1;

        // add the rotation matrix to the transformation matrix
        for (size_t idx = 0; idx < 3; ++idx)
            for (size_t idy = 0; idy < 3; ++idy)
                Tm (idx,idy) = rmat.at<double>(idx,idy);

        Tm(0,3) = (float)tvec.at<double>(0);
        Tm(1,3) = (float)tvec.at<double>(1);
        Tm(2,3) = (float)tvec.at<double>(2);


        float currentRotation = atan2(Tm(0,1), Tm(0,0)) * 180 / M_PI;
        stringstream ss;
        ss << currentRotation;
        putText(image, ss.str(), Point(5,30), CV_FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 255, 255), 1, CV_AA, false);
        imshow("image", image);
        waitKey(1);
        return Tm;
    }
    else // if pattern was not found just return a matrix full of zeros
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

    int count1 = 0;

    for (size_t idx = 0; idx < width; ++idx)
        for (size_t idy = 0; idy < height; ++idy)
        {
            int b = cloud.points[count1].b;
            int g = cloud.points[count1].g;
            int r = cloud.points[count1].r;
            newImage.at<cv::Vec3b>(idx, idy)[0] = (unsigned char)b;
            newImage.at<cv::Vec3b>(idx, idy)[1] = (unsigned char)g;
            newImage.at<cv::Vec3b>(idx, idy)[2] = (unsigned char)r;
            ++count1;
        }

    return newImage;
}

void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &cam_info)
{
    Mat D;
    CameraInfo ci = *cam_info;
    d_mK = cv::Mat(3, 3, CV_32FC1);

    for (int i = 0; i < 9; i++)
        d_mK.at<float>(i / 3, i % 3) = ci.K[i];

    D = cv::Mat(ci.D.size(), 1, CV_32FC1);

    for (size_t i = 0; i < ci.D.size(); i++)
        D.at<float>(i) = ci.D[i];

    d_cam_info_sub.shutdown(); // don't need this anymore, so shut it down
}

void onMouseClick(int event, int x, int y, int, void*)
{
	if (event != cv::EVENT_LBUTTONDOWN)
		return;

	// do pointCloud saving here
	stringstream ss;
	ss << dirname <<  "/cloud_" << count2 << ".pcd";

	io::savePCDFile(ss.str(), *cloud);

	ROS_INFO_STREAM("Saving cloud_" << count2 << " in " << dirname);
	++count2;
}

void onMouseClick2(int event, int x, int y, int, void*)
{
	if (event != cv::EVENT_LBUTTONDOWN)
		return;

	// do pointCloud saving here
	stringstream ss;
	ss << dirname <<  "/image_" << count2 << ".jpg";

	cv::imwrite(ss.str(), imageObject);

	ROS_INFO_STREAM("Saving image_" << count2 << " in " << dirname);
	++count2;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object modeler");
    ros::NodeHandle nh("object_modeler");

    ros::Subscriber cloudSub;

    cv::namedWindow("Controls", 1);
	cv::namedWindow("image", 1);
	cv::namedWindow("object2d", 1);

	count2 = 0;

	nh.getParam("save_folder", dirname);
	mkdir(dirname.c_str(),S_IRWXU);

	cout << "Saving files in " << dirname << endl;
	cout << "Files will be overwritten!!\n";

	zeroPointSlider = 0;
	zeroPointSliderMax = 100;
	zeroPointCutoff = 0;
	cv::createTrackbar("zero point cutoff", "Controls", &zeroPointSlider, zeroPointSliderMax, onZeroPointSliderChange);
	cv::setMouseCallback("image", onMouseClick);
	cv::setMouseCallback("object2d", onMouseClick2);

	cv::setTrackbarPos("zero point cutoff", "Controls", 50);

	bottomSlider = 0;
	bottomSliderMax = 500;
	bottomCutoff = 0;
	cv::createTrackbar("bottom cutoff", "Controls", &bottomSlider, bottomSliderMax, onBottomSliderChange);
	cv::setTrackbarPos("bottom cutoff", "Controls", 100);

	leftSlider = 0;
	leftSliderMax = 500;
	cv::createTrackbar("left cutoff", "Controls", &leftSlider, leftSliderMax, onLeftSliderChange);
	cv::setTrackbarPos("left cutoff", "Controls", 100);

	rightSlider = 0;
	rightSliderMax = 500;
	cv::createTrackbar("right cutoff", "Controls", &rightSlider, rightSliderMax, onRightSliderChange);
	cv::setTrackbarPos("right cutoff", "Controls", 100);

	frontSlider = 0;
	frontSliderMax = 500;
	cv::createTrackbar("front cutoff", "Controls", &frontSlider, frontSliderMax, onFrontSliderChange);
	cv::setTrackbarPos("front cutoff", "Controls", 100);

	backSlider = 0;
	backSliderMax = 500;
	cv::createTrackbar("back cutoff", "Controls", &backSlider, backSliderMax, onBackSliderChange);
	cv::setTrackbarPos("back cutoff", "Controls", 100);


    cloudSub = nh.subscribe<PCLPointCloud>("/xtion/depth_registered/points", 1, pcCallBack);
    d_cam_info_sub = nh.subscribe<sensor_msgs::CameraInfo>("/xtion/camera_info", 1, camInfoCallback);
    cloudPub = nh.advertise<PCLPointCloud>("object_modeler/model", 1);





    ros::spin();
}
