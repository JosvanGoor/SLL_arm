/*
 * headSpinner.h
 *
 *  Created on: Sep 1, 2015
 *      Author: rik
 */

#ifndef SOURCE_DIRECTORY__HEAD_SPIN_INCLUDE_HEADSPINNER_H_
#define SOURCE_DIRECTORY__HEAD_SPIN_INCLUDE_HEADSPINNER_H_


#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <dynamixel_controllers/SetSpeed.h>
#include <math.h>
#include <dynamixel_msgs/JointState.h>


class HeadSpinner
{
	ros::NodeHandle d_nh;

	ros::ServiceClient tiltClient;
	ros::ServiceClient panClient;

	dynamixel_controllers::SetSpeed tiltSpeed;
	dynamixel_controllers::SetSpeed panSpeed;

	ros::Publisher tiltPub;
	ros::Publisher panPub;

	std_msgs::Float64 tiltPos;
	std_msgs::Float64 panPos;

	ros::Subscriber subStart;

	ros::Publisher pubGetImage;
	ros::Subscriber subImageReceived;
	bool d_imageTaken;
	bool d_panIsMoving;
	ros::Subscriber subPanMoving;

	public:
		HeadSpinner(ros::NodeHandle nh);

	private:
		double degreeToRad(double deg);
		void startCallback(const std_msgs::BoolConstPtr &start);
		void takenImageCallback(const std_msgs::BoolConstPtr &received);
		void panMovingCallback(const dynamixel_msgs::JointStateConstPtr &state);
		void moveHead();
		void setSpeed(float speed);
		void moveHeadToStart();
};


#endif /* SOURCE_DIRECTORY__HEAD_SPIN_INCLUDE_HEADSPINNER_H_ */
