#ifndef __INCLUDED_ROBOTDRIVER_H_
#define __INCLUDED_ROBOTDRIVER_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <iosfwd>

class RobotDriver
{
private:
	//! The node handle we'll be using
	ros::NodeHandle nh_;
	//! We will be publishing to the "cmd_vel" topic to issue commands
	ros::Publisher cmd_vel_pub_;
	//! We will be listening to TF transforms as well
	tf::TransformListener listener_;
	//Service client for borg memory
	ros::ServiceClient client_reader;
	ros::ServiceClient client_writer;

	//Rate of this class
	ros::Rate loop_rate;

	//Holds the time for last command
	double last_command;

public:
	//! ROS node initialization
	RobotDriver(ros::NodeHandle &nh);

	//! Drive forward a specified distance based on odometry information
	bool driveForwardOdom(double distance);

	//!Turn a specified Radians based on odometry information
	bool turnOdom(bool clockwise, double radians);

	bool jsonParser(std::string &type, bool &clockWise, double &amount);
};

#endif
