#include <iostream>
#include <ros/ros.h>

#include <tf/transform_listener.h>

// PointCloud includes
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/features/feature.h>
#include <pcl_ros/segmentation/sac_segmentation.h>
#include <pcl_ros/segmentation/extract_clusters.h>
#include <pcl_ros/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>


#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

// boost includes
#include <boost/algorithm/string/predicate.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>

#include <alice_msgs/MemorySrv.h>
#include <alice_msgs/MemoryReadSrv.h>
#include <jsoncpp/json/json.h>


using namespace std;
using namespace ros;
using namespace pcl;

using boost::property_tree::ptree;

typedef PointXYZRGB Point;
typedef PointCloud<Point> PCLPointCloud;
typedef PointCloud<Point>::Ptr PCLPointCloudPtr;

#ifdef __CDT_PARSER__
    #define foreach(a, b) for(a : b)
#else
    #define foreach(a, b) BOOST_FOREACH(a, b)
#endif

double last_command;

tf::TransformListener *tfTransformer;
Publisher cloud_pub;
Publisher cmd_vel;
Publisher tilt;
Publisher pan;

//Service client for borg memory
ros::ServiceClient client_reader;
ros::ServiceClient client_writer;

ros::Subscriber depth_sub;

size_t bufferClear;

double currentAngle;
bool process;

double currentX;
double currentY;

void writeCompleted();

bool firstTimeInit = true;

bool align = true;

void moveHead()
{
	std_msgs::Float64 tilt_angle;
	tilt_angle.data = 0.9f;
	std_msgs::Float64 pan_angle;
	pan_angle.data = 0.0f;
	tilt.publish(tilt_angle);
	pan.publish(pan_angle);
}

void odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
    currentX = msg->pose.pose.position.x; // x position
    currentY = msg->pose.pose.position.y; // y position
    double w = msg->pose.pose.orientation.w; // for the rotation

    currentAngle = 2 * acos(w) * (180.0f / M_PI); // the current rotation angle in degrees
}

bool turn(float angle)
{
	geometry_msgs::Twist base_cmd;
	base_cmd.linear.x = base_cmd.angular.z = 0; // init at 0 to be sure

	float turnSpeed = 0.2f; // turn speed

	if (angle < 0) // turn right, else just turn left
		turnSpeed *= -1.0f;

	float beginAngle = currentAngle;
	base_cmd.angular.z = turnSpeed;

	ros::Rate rate(100);
	float distance = 0.0f;
	float prevAngle = 0.0f;
	bool first = true;

	while (distance < fabs(angle))
	{
		float angleStep = 0.0f;

		if (first)
			angleStep = 0.0f;
		else
			angleStep = fabs(currentAngle - prevAngle);
		first = false;
		prevAngle = currentAngle;
		distance += angleStep;
		ROS_INFO_STREAM("TURNING");
		cmd_vel.publish(base_cmd);
		rate.sleep();
		ros::spinOnce();
	}

	// stop turning
	base_cmd.angular.z = 0.0f;
	cmd_vel.publish(base_cmd);
	ROS_INFO_STREAM(" ");

	moveHead();
	ros::Duration(1.0).sleep();

	return true;
}

void moveForward(float meter)
{
	float startX = currentX;
	float startY = currentY;

	geometry_msgs::Twist base_cmd;
	base_cmd.linear.x = base_cmd.angular.z = 0; // init at 0 to be sure

	float driveSpeed = 0.1;

	base_cmd.linear.x = driveSpeed;
	cmd_vel.publish(base_cmd);

	float distance = 0.0f;

	ros::Rate rate(100);

	while (distance <= meter) // keep driving until distance is enough meters
	{
	//	ROS_INFO_STREAM("Start: " << startX << ", " << startY);
	//	ROS_INFO_STREAM("Current: " << currentX << ", " << currentY);
		distance = fabs(sqrt(pow(currentX - startX, 2) + pow(currentY - startY,2)));
	//	ROS_INFO_STREAM("Distance driven: " << distance << ", goal: " << meter);
		cmd_vel.publish(base_cmd);
		rate.sleep();
		ros::spinOnce();
	}

	// done stop driving
	base_cmd.linear.x = 0.0f;
	cmd_vel.publish(base_cmd);
}

float getAngle(float baseX, float baseY, float x, float y)
{
	float dot = baseX * x + baseY * y;
	float magA = sqrt(pow(baseX, 2) + pow(baseY, 2));
	float magB = sqrt(pow(x, 2) + pow(y, 2));

	float angle = acos(dot / (magA * magB)) * (180.0 / M_PI);
	return angle;
}

float getDistance(float x, float y)
{
	float distance = sqrt(pow(x,2) + pow(y,2));
	return distance;
}


bool busy = false;

void pcCallback(const sensor_msgs::PointCloud2ConstPtr &cloudMsg)
{
	if (busy)
		return;

	if (process == true)
	{
		tfTransformer->waitForTransform("/base_link", "/front_xtion_link", ros::Time::now(), ros::Duration(0.05));

		if (!tfTransformer->canTransform("/base_link", cloudMsg->header.frame_id, cloudMsg->header.stamp))
		{
			ROS_INFO_STREAM("No transform found");
			return;
		}

		PCLPointCloudPtr pointcloud(new PCLPointCloud);
		sensor_msgs::PointCloud2 cloudMsg_transformed;
		pcl_ros::transformPointCloud("/base_link", *cloudMsg, cloudMsg_transformed, *tfTransformer);

		fromROSMsg(cloudMsg_transformed, *pointcloud);

		vector<int> ind;
		removeNaNFromPointCloud(*pointcloud, *pointcloud, ind);

		PassThrough<Point> passthrough_filter;
		passthrough_filter.setInputCloud(pointcloud);
		passthrough_filter.setFilterFieldName("z");
		passthrough_filter.setFilterLimits(0.50, 0.9);
		passthrough_filter.filter(*pointcloud);
		
		passthrough_filter.setInputCloud(pointcloud);
		passthrough_filter.setFilterFieldName("x");
		passthrough_filter.setFilterLimits(0.0, 1.2);
		passthrough_filter.filter(*pointcloud);

		ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		PointIndices::Ptr inliers (new pcl::PointIndices);
		// Create the segmentation object
		SACSegmentation<Point> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold (0.02);  // 0.015

		seg.setInputCloud(pointcloud);
		seg.segment(*inliers, *coefficients);

		ExtractIndices<Point> extract;

		extract.setInputCloud(pointcloud);
		extract.setIndices(inliers);

		PCLPointCloudPtr plane(new PCLPointCloud);
		extract.setNegative(false);  // keep the planes
		extract.filter(*plane);

		extract.setNegative(true); // true means remove planes
		extract.filter(*pointcloud);

		pointcloud->header.frame_id = "front_xtion_link";
		cloud_pub.publish(plane);

		if (align)
		{
			vector<pcl::PointIndices> cluster;
			pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);

			tree->setInputCloud(plane);
			EuclideanClusterExtraction<Point> ec;

			ec.setClusterTolerance(0.02);       // in meters
			ec.setMinClusterSize(100);       // Minimal points that must belong to a cluster
			ec.setMaxClusterSize(400000);      // Maximal points that must belong to a cluster
			ec.setSearchMethod(tree);
			ec.setInputCloud(plane);
			ec.extract(cluster);

			ROS_INFO_STREAM("Planes found: " << cluster.size());

			if (cluster.size() == 0) // no planes found?
				return;

			for (size_t idx = 0; idx < cluster.size(); ++idx)
			{
				PCLPointCloudPtr objectCloud(new PCLPointCloud);
				pcl::PointIndices point_indices = cluster.at(idx);

				foreach (int index, point_indices.indices)
				{
					Point p = plane->points[index];
					objectCloud->points.push_back(p);
				}

				Point minPt, maxPt;
				getMinMax3D(*objectCloud, minPt, maxPt);

				Point searchPoint;

               // ROS_INFO_STREAM(fabs(maxPt.z - minPt.z) << ", " << fabs(maxPt.y - minPt.y) << ", " << fabs(maxPt.x - minPt.x));
				if (fabs(maxPt.z - minPt.z) < 0.09f && fabs(maxPt.y - minPt.y) > 0.4f && fabs(maxPt.x - minPt.x) > 0.4f) // plane can not have a large height
				{


					pcl::KdTreeFLANN<Point> kdtree;
					kdtree.setInputCloud (objectCloud);

					//determine the closest 100 points
					int K = 5;
					vector<int> pointIdxNKNSearch(K);
					vector<float> pointNKNSquaredDistance(K);

					searchPoint.x = 0.0f;
					searchPoint.y = 0.30f;
					searchPoint.z = 0.0f;

					float xOne = 0.0f;
					float yOne = 0.0f;

					if (kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
					{
						for (size_t idb = 0; idb < 5; ++idb)
						{
							xOne += objectCloud->points[pointIdxNKNSearch[idb]].x;
							yOne += objectCloud->points[pointIdxNKNSearch[idb]].y;
						}

						xOne /= 5;
						yOne /= 5;
					}

					searchPoint.x = 0.0f;
					searchPoint.y = -0.30f;
					searchPoint.z = 0.0f;

					float xTwo = 0.0f;
					float yTwo = 0.0f;

					if (kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
					{
						for (size_t idb = 0; idb < 5; ++idb)
						{
							xTwo += objectCloud->points[pointIdxNKNSearch[idb]].x;
							yTwo += objectCloud->points[pointIdxNKNSearch[idb]].y;
						}

						xTwo /= 5;
						yTwo /= 5;
					}


					ROS_INFO_STREAM(xOne <<"," << yOne << ", " << xTwo <<", " << yTwo << ", " << xTwo - xOne);

					//xOne -= xOne;
					//yOne -= yOne;
					//xTwo = xTwo - xOne;
					//yTwo -= yOne;

					float xG = xTwo - xOne;
					float yG = yTwo - yOne;

					float baseX = 1.0f; // vector 2 meters in front
					float baseY = 0.0f;

					ROS_INFO_STREAM(xOne << ", " << yOne);
					ROS_INFO_STREAM(xG << ", " <<yG);
					float angle = getAngle(baseX, baseY, xG, yG);
					ROS_INFO_STREAM("Angle: " << angle);

					if (angle <= 95.0f and angle >= 85.0f) // no rotation needed
					{
						align = false;
						ROS_INFO_STREAM("DONE");
						return;
					}

					if (angle > 95.0f) // 5 degree error
					{
						angle -= 90.0f;
						angle *= -1;
					}
					else if (angle < 85.0f) // 5 degree error
					{
						angle = fabs(angle - 90.0f);
						//angle *= -1;
					}

					ROS_INFO_STREAM("ROTATION ANGLE: " << angle);
					align = false;
					busy = true;
					ROS_INFO_STREAM("ALIGN...");
					busy = !turn(angle);
				}
			}

			return;

		}


		vector<pcl::PointIndices> cluster;
		pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);

		tree->setInputCloud(pointcloud);
		EuclideanClusterExtraction<Point> ec;

		ec.setClusterTolerance(0.05);       // in meters
		ec.setMinClusterSize(100);       // Minimal points that must belong to a cluster
		ec.setMaxClusterSize(40000);      // Maximal points that must belong to a cluster
		ec.setSearchMethod(tree);
		ec.setInputCloud(pointcloud);
		ec.extract(cluster);

		if (cluster.size() == 0)
		{
			ROS_INFO_STREAM("Cannot find any clusters");
			return;
		}

		if (cluster.size() != 0)
		{
			ROS_INFO_STREAM("Clusters: " << cluster.size());
			float avgY = 0.0f;
			float avgX = 0.0f;
			size_t correctObjCount = 0;

			for (size_t idx = 0; idx < cluster.size(); ++idx)
			{
				PCLPointCloudPtr objectCloud(new PCLPointCloud);
				pcl::PointIndices point_indices = cluster.at(idx);

				foreach (int index, point_indices.indices)
				{
					Point p = pointcloud->points[index];
					objectCloud->points.push_back(p);
				}

				Point minPt, maxPt;
				Eigen::Matrix<float, 4, 1> cenPoint;
				getMinMax3D(*objectCloud, minPt, maxPt);

				if ( fabs(maxPt.z - minPt.z) <= 0.20f && fabs(maxPt.z - minPt.z) > 0.04) // objects are not higher then 20cm
				{
					compute3DCentroid(*objectCloud, cenPoint);
					ROS_INFO_STREAM("Point: " << cenPoint[0] << ", " << cenPoint[1]);
					ROS_INFO_STREAM("Min/max: " << minPt.x << ", " << maxPt.x << ", " << minPt.y <<", " << maxPt.y);

					avgX += cenPoint[0];
					avgY += cenPoint[1];
					++correctObjCount;
				}
			}

			avgX /= correctObjCount; // the middle X position
			avgY /= correctObjCount; // the middle Y position

			ROS_INFO_STREAM("avgX: " << avgX <<", avgY: " << avgY);

			float moveDistance = 0.55f;

			// vector right in front of base_link
			float baseX = 1.0f;
			float baseY = 0.0f;

			float x = avgX - moveDistance;
			float y = avgY;
			ROS_INFO_STREAM("goto point: " << x << ", " << y);
			float angle = getAngle(baseX, baseY, x, y);
			float distance = getDistance(x,y);

			float xObj = avgX;
			float yObj = 0.0f;
			float baseXr = -x;
			float baseYr = -y;

			float returnAngle = 180.0f - getAngle(baseXr, baseYr, xObj, yObj);

			process = false;

			if (y <= 0.0f) // point is on the right so rotate right
				angle *= -1;
			else
				returnAngle *= -1; // return rotation needs to be opposite rotation of first rotation

			turn(angle);
			moveForward(distance);
			turn(returnAngle);

			ros::Duration(2.0).sleep(); // sleep for 2 seconds
			writeCompleted();

		}

	}

}

void writeCompleted()
{
	alice_msgs::MemorySrv srvWrite;
	srvWrite.request.timestamp = ros::Time::now();
	double secs =ros::Time::now().toSec();
	srvWrite.request.name = "completed_movement";
	char jsonmsg[255];
	sprintf(jsonmsg, "{\"time\": %f, \"done\": %d}", secs ,true);
	srvWrite.request.json = std::string(jsonmsg);
	client_writer.call(srvWrite);
}

void resetMessage()
{
	alice_msgs::MemorySrv srvWrite;
	srvWrite.request.timestamp = ros::Time::now();
	double secs =ros::Time::now().toSec();
	srvWrite.request.name = "move_to_position";
	char jsonmsg[255];
	sprintf(jsonmsg, "{\"time\": %f, \"move\": %d}", secs ,false);
	srvWrite.request.json = std::string(jsonmsg);
	client_writer.call(srvWrite);

}

bool checkMemory()
{
	if (process == false) // only need to check this when not processing already
	{
		alice_msgs::MemoryReadSrv srv;
		srv.request.function = "get_last_observation";
		srv.request.timestamp = ros::Time::now();
		srv.request.name = "move_to_position";
		srv.request.params = "";

		// Make the call
		if (not client_reader.call(srv))
		{
			ROS_WARN_ONCE("No connection\n");
			return false;
		}

		// Read message
		string msg = srv.response.json;

		// Let's parse it
		Json::Value root;
		Json::Reader reader;
		bool parsedSuccess = reader.parse(msg, root, false);


		if (not parsedSuccess)
		{
			// Report failures and their locations
			// in the document.
			ROS_ERROR_STREAM("Failed to parse JSON\n" << reader.getFormatedErrorMessages() << '\n');
			return true;
		}

		//Extracting Parts
		Json::Value const time = root["time"];
		double cmd_time;
		string cmd;

		if (not time.isNull())
		{
			cmd_time = time.asDouble();

			//Only continue if the command is new
			if (cmd_time <= last_command)
				return true;
			last_command = cmd_time;
		}

		Json::Value const command = root["move"];

		bool moveRobot = false;

		if (not command.isNull())
		{
			moveRobot = command.asBool();

			if (moveRobot)
			{
				bufferClear = 0;
				ROS_INFO_STREAM("MOVE ROBOT");
				align = true;
				process = true; // start the processing of pointcloud and movement
			}
		}

		resetMessage(); // reset borg memory to activate this function
	}

	return true;
}


int main(int argc, char **argv)
{
	process = false;
	ros::init(argc, argv, "moveing_Alice");
	ros::NodeHandle node_handle("");

	depth_sub = node_handle.subscribe<sensor_msgs::PointCloud2>("front_xtion/depth_registered/points", 1, pcCallback);
	ros::Subscriber odom = node_handle.subscribe<nav_msgs::Odometry>("odom", 1, odomCB);
	cloud_pub = node_handle.advertise<PCLPointCloud>("tempOutput", 1);

	tilt = node_handle.advertise<std_msgs::Float64>("tilt_controller/command", 1);
	pan = node_handle.advertise<std_msgs::Float64>("pan_controller/command", 1);

	cmd_vel = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    // Create listener and set global pointer
    tf::TransformListener temp_listener;
    tfTransformer = &temp_listener;

    // loop
	ros:Rate loopRate(20);

	while (ros::ok())
	{
		if (!checkMemory())
		{
			client_writer = node_handle.serviceClient<alice_msgs::MemorySrv>("memory");
			client_reader = node_handle.serviceClient<alice_msgs::MemoryReadSrv>("memory_read");
		}

		loopRate.sleep();
		ros::spinOnce();
	}


}
