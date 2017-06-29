#ifndef H_RECORDER
#define H_RECORDER

#include <object_modeler/utils.h>


class Recorder
{
	// nodehandle
	ros::NodeHandle d_nh;

	// subscribers
	ros::Subscriber d_depth_sub;
	ros::Subscriber d_cam_info_sub;

	// publishers
	ros::Publisher d_cloud_pub;
	ros::Publisher d_cloud_pub2;

	// bools
	bool d_bCameraInfoReady;
	bool d_bRetryCluster;

	// Matrix or images
	Mat d_mK;

	// floats
	float d_fPrevRotation;

	// doubles
	double d_dStepSize;

	// vector
	vector<PCLPointCloud> d_vObject;

	// size_t 
	size_t d_stNrOfSegments;


	// slider params
	int bottomSlider;
	int bottomSliderMax;


	public:
		Recorder(ros::NodeHandle &nh);

	private:
		void pcCallBack(const PCLPointCloud::ConstPtr &cloudMsg);
		void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &cam_info);
		void draw(cv::Mat& drawImage, const cv::Mat& K, const cv::Mat& R, const cv::Mat T);
		void segmentPointcloud(PCLPointCloudPtr &cloud, Eigen::Matrix4f transformMatrix);
		bool cluster(PCLPointCloudPtr &cloud, PCLPointCloudPtr &outCloud, Eigen::Matrix4f transformMatrix);

		Mat makeImage(PCLPointCloud cloud);

		Eigen::Matrix4f getTransformation(Mat image);

		vector<cv::Point3f>  calcChessboardCorners(Size boardSize, float squareSize, Point3f offset = Point3f());

		pcl::PointCloud<SHOT352>::Ptr databasialize(PCLPointCloud input);
		void sample(PCLPointCloud inputImg, PointCloudOut &output);
		pcl::PointCloud<Normal>::Ptr computeNormal(PCLPointCloud inputCloud);
		void getSHOTDescriptors(PCLPointCloud inputImg, vector<int> keypoints,pcl::PointCloud<Normal>::Ptr normals,pcl::PointCloud<SHOT352> &output);
		vector<double> getSize(PCLPointCloud inputCld);
		void findDims(vector<double> xs, vector<double> ys, vector<double> zs, string dirname);
		void callbackBottomSlider(int, void*);


		

};

#endif
