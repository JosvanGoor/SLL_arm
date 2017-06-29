#ifndef H_RECORDER
#define H_RECORDER

#include <modeler/utils.h>
#include <modeler/reconstructer.h>


class Recorder
{
	// nodehandle
	ros::NodeHandle d_nh;

	// subscribers
	ros::Subscriber d_depth_sub;
	ros::Subscriber d_cam_info_sub;

	// publishers
	ros::Publisher d_cloud_pub;

	// bools
	bool d_bCameraInfoReady;
	bool d_bRetryCluster;

	// Matrix or images
	Mat d_mK;

	// floats
	float d_fPrevRotation;
	float d_fDegrees;

	// vector
	vector<PCLPointCloud> d_vObject;

	// size_t 
	size_t d_stNrOfSegments;

	// Reconstructer
	Reconstructer d_reconstructer;

	string d_objectName;

	public:
		Recorder();

	private:
		void pcCallBack(const PCLPointCloud::ConstPtr &cloudMsg);
		void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &cam_info);
		void draw(cv::Mat& drawImage, const cv::Mat& K, const cv::Mat& R, const cv::Mat T);
		void segmentPointcloud(PCLPointCloudPtr &cloud, Eigen::Matrix4f transformMatrix);
		bool cluster(PCLPointCloudPtr &cloud, PCLPointCloudPtr &outCloud, Eigen::Matrix4f transformMatrix);

		Mat makeImage(PCLPointCloud cloud);

		Eigen::Matrix4f getTransformation(Mat image);

		vector<cv::Point3f>  calcChessboardCorners(Size boardSize, float squareSize, Point3f offset = Point3f());
		

};

#endif