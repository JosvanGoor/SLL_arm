#ifndef _MICO_GRAPSING_ACTION_H_
#define _MICO_GRASPING_ACTION_H_

#include <ros/ros.h>
#include "jaco_driver/jaco_comm.h"

#include <actionlib/server/simple_action_server.h>
#include <jaco_msgs/graspingLocation.h>
#include <jaco_msgs/graspingAction.h>
#include <arm_planner/pathAction.h>

#include <vector>

using namespace std;

namespace jaco
{
	class GraspingActionServer
	{
		enum STATUS
		{
			Init = 0,
			NoObjectNav,
			ObjectNavLeft,
			ObjectNavRight
		};

		STATUS d_status;

		public:
			GraspingActionServer(JacoComm &, ros::NodeHandle &n);
			void execute(const jaco_msgs::graspingGoalConstPtr &goal);
			void executePath(const arm_planner::pathGoalConstPtr &path);

		private:
			JacoComm &arm;
    		actionlib::SimpleActionServer<jaco_msgs::graspingAction> as;
    		actionlib::SimpleActionServer<arm_planner::pathAction> asPath;
    		float d_fObjectHeight;
    		void checkFingers(FingerAngles fingers);
    		void checkWaypoint(JacoPose wayPoint);
    		void moveToLeft(float z);
    		void moveToRight(float z);
    		void movePath(vector<arm_planner::Position> waypoints);
	};
}







#endif
