/*
 * turnodom.cc
 *
 *  Created on: 14 jan. 2014
 *      Author: Amir Shantia
 */
#include "robotdriver.ih"

bool RobotDriver::turnOdom(bool clockwise, double radians)
{
	while (radians < 0)
		radians += 2 * M_PI;
	while (radians > 2 * M_PI)
		radians -= 2 * M_PI;

	//wait for the listener to get the first message
	try
	{
	listener_.waitForTransform("base_link", "odom", ros::Time(0),
			ros::Duration(1.0));
	} catch (tf::TransformException &ex)
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
	//the command will be to turn at 0.75 rad/s
	base_cmd.linear.x = base_cmd.linear.y = 0.0;
	base_cmd.angular.z = 0.75;
	if (clockwise)
		base_cmd.angular.z = -base_cmd.angular.z;

	//the axis we want to be rotating by
	tf::Vector3 desired_turn_axis(0, 0, 1);
	if (!clockwise)
		desired_turn_axis = -desired_turn_axis;

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
		tf::Transform relative_transform = start_transform.inverse()
				* current_transform;

		tf::Vector3 actual_turn_axis =
				relative_transform.getRotation().getAxis();
		double angle_turned = relative_transform.getRotation().getAngle();


		if (fabs(angle_turned) < 1.0e-2)
			continue;

		if (actual_turn_axis.dot(desired_turn_axis) < 0)
			angle_turned = 2 * M_PI - angle_turned;

		ROS_DEBUG_STREAM("Angle Turned: " << angle_turned << "\n Desired Radians: " << radians << '\n');
		if (angle_turned > radians)
		{
			ROS_DEBUG("Turn Finished\n");
			base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
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

}

