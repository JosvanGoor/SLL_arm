#ifndef H_MICOARM
#define H_MICOARM

#include <Kinova.API.CommLayerUbuntu.h>
#include <KinovaTypes.h>
#include <dlfcn.h>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <assert.h>
#include <ros/ros.h>
#include <math.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <boost/foreach.hpp>
#include <vector>
#include <string>
#include <sensor_msgs/JointState.h>


class MicoArm
{
	//Handle for the library's command layer.
	void *commandLayer_handle;

	//Function pointers to the functions we need
	int (*initAPI)();
	int (*closeAPI)();
	int (*sendAdvanceTrajectory)(TrajectoryPoint command);
	int (*getDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
	int (*setActiveDevice)(KinovaDevice device);
	int (*moveHome)();
	int (*initFingers)();
	int (*getAngularCommand)(AngularPosition &);
	int (*getCartesianPosition)(CartesianPosition &);

	ros::Timer status_timer;
	ros::Publisher joint_state_publisher;

	// Trajectory action server
	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> micoTrajectoryServer;

public:
	MicoArm(ros::NodeHandle nh);
	~MicoArm();
	void sendUpRight();
	void sendVelocity(float speed1, float speed2, float speed3, float speed4, float speed5, float speed6);
	void getJointPositions(AngularPosition &joints);
	void getRealJointPositions(AngularPosition &joints);
	void getEEFPosition(CartesianPosition &eef);
	void actionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

private:
	void limitJoint(float &joint);
	float toDegrees(float x);
	float toRadian(float x);
	bool checkDistance(float current, float goal);
	float velocity(float acceleration, float time);
	bool isZeroVelocity(trajectory_msgs::JointTrajectoryPoint point);
	bool isAlmostZeroVelocity(TrajectoryPoint point);
	void createMicoTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);
	void statusTimer(const ros::TimerEvent&);
	void publishJointAngles();
};

#endif
