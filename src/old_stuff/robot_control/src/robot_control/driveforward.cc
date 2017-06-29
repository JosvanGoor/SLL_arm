/*
 * driveforward.cc
 *
 *  Created on: 14 jan. 2014
 *      Author: Amir Shantia
 */

#include "robotdriver.ih"

bool RobotDriver::driveForwardOdom(double distance)
{
//wait for the listener to get the first message
	try
	{
	listener_.waitForTransform("base_link", "odom", ros::Time(0),
			ros::Duration(1.0));
	} catch (tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
		return false;
	}

//we will record transforms here
	tf::StampedTransform start_transform;
	tf::StampedTransform current_transform;

//record the starting transform from the odometry to the base frame
	listener_.lookupTransform("base_link", "odom", ros::Time(0),
			start_transform);

//we will be sending commands of type "twist"
	geometry_msgs::Twist base_cmd;
//the command will be to go forward at 0.25 m/s
	base_cmd.linear.y = base_cmd.angular.z = 0;
	base_cmd.linear.x = 0.25;

	bool done = false;
	while (!done && nh_.ok())
	{
		//send the drive command
		cmd_vel_pub_.publish(base_cmd);
		loop_rate.sleep();
		//get the current transform
		try
		{
			listener_.lookupTransform("base_link", "odom",
					ros::Time(0), current_transform);
		} catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			break;
		}
		//see how far we've traveled
		tf::Transform relative_transform = start_transform.inverse()
				* current_transform;
		double dist_moved = relative_transform.getOrigin().length();

		if (dist_moved > distance)
		{
			ROS_DEBUG("Stopping the robot");
			base_cmd.linear.y = base_cmd.angular.z = base_cmd.linear.x = 0;
		    cmd_vel_pub_.publish(base_cmd);
			done = true;
		}
	}

	borg_pioneer::MemorySrv srv;
	srv.request.timestamp = ros::Time::now();
	srv.request.name = "drive_forward";
	char jsonmsg[255];
	sprintf(jsonmsg, "{\"state\": %d}", done);
	srv.request.json = std::string(jsonmsg);
	client_writer.call(srv);

	return done;
}

