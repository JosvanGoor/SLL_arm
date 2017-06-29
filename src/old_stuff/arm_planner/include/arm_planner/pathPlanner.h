#ifndef _H_PATHPLANNER
#define _H_PATHPLANNER

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_planner/goalAction.h>
#include <arm_planner/pathAction.h>
#include <arm_planner/Position.h>

#include <arm_planner/ikChecker.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

// boost includes
#include <boost/algorithm/string/predicate.hpp>
#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>

using namespace std;
using namespace pcl;

using boost::property_tree::ptree;

typedef PointXYZ Point;
typedef PointCloud<Point> PCLPointCloud;
typedef PointCloud<Point>::Ptr PCLPointCloudPtr;

#ifdef __CDT_PARSER__
    #define foreach(a, b) for(a : b)
#else
    #define foreach(a, b) BOOST_FOREACH(a, b)
#endif


class PathPlanner
{
	actionlib::SimpleActionServer<arm_planner::goalAction> as;
	actionlib::SimpleActionClient<arm_planner::pathAction> pathClient;
	IKChecker d_ikChecker;
	PCLPointCloudPtr d_grid; // the grid point cloud

	KdTreeFLANN<Point> kdtree;

	size_t counter;


	public:
		PathPlanner(ros::NodeHandle &n);
		void execute(const arm_planner::goalGoalConstPtr &goal);

	private:
		bool makePath(float x, float y, float z);
		bool makePathTest(float x, float y, float z);
		arm_planner::Position createPoint(float x, float y, float z, float thetaX, float thetaY, float thetaZ = 0.0f);
		void createGrid();
		vector<int> checkPointsInRange(Point search);
		bool findPath(vector<arm_planner::Position> &vPoints);
		float getDistance(arm_planner::Position sPoint, arm_planner::Position gPoint);
		void addPoint(vector< vector<arm_planner::Position> > &vPath, vector<arm_planner::Position> vWaypoints, arm_planner::Position nPoint, const arm_planner::Position &gPoint, float prevDistance = 99.0f);
		void addInbetweens(vector<arm_planner::Position> &vPoints);
		arm_planner::Position getStartPosition();
		void moveToLeft(vector<arm_planner::Position> &vPath);
		void moveToRight(vector<arm_planner::Position> &vPath);
		void moveNavLeft(vector<arm_planner::Position> &vPath);
		void moveNavRight(vector<arm_planner::Position> &vPath);
};

#endif
