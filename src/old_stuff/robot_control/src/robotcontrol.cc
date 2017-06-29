/*
 * robotcontrol.cc
 *
 *  Created on: 14 jan. 2014
 *      Author: Amir Shantia
 */
#include <robot_control/robotdriver.h>
#include <borg_pioneer/MemoryReadSrv.h>
#include <ros/console.h>

#include <string>
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
	//init the ROS node
	ros::init(argc, argv, "robot_driver");
	ros::NodeHandle nh;

	RobotDriver driver(nh);

	while (ros::ok())
	{
		string type;
		bool clockWise;
		double amount;
		if (driver.jsonParser(type, clockWise, amount))
		{
			if (type == "move")
			{
				ROS_DEBUG("Received Move Command");
				driver.driveForwardOdom(amount);
			}
			else
			{
				ROS_DEBUG_STREAM("Received Turn Command with clockwise as " << clockWise << "and amount as "
						<< amount << '\n');
				driver.turnOdom(clockWise, amount);
			}
		}
	}
}

