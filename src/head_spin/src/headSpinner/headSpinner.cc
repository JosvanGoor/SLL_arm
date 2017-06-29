/*
 * headSpinner.cc
 *
 *  Created on: Sep 1, 2015
 *      Author: rik
 */


#include <headSpinner.h>

HeadSpinner::HeadSpinner(ros::NodeHandle nh) :
	tiltClient(nh.serviceClient<dynamixel_controllers::SetSpeed>("tilt_controller/set_speed")),
	panClient(nh.serviceClient<dynamixel_controllers::SetSpeed>("pan_controller/set_speed")),
	tiltPub(nh.advertise<std_msgs::Float64>("tilt_controller/command", 1)),
	panPub(nh.advertise<std_msgs::Float64>("pan_controller/command", 1)),
	subStart(nh.subscribe<std_msgs::Bool>("headSpinner/start", 1, &HeadSpinner::startCallback, this)),
	subPanMoving(nh.subscribe<dynamixel_msgs::JointState>("/pan_controller/state", 1, &HeadSpinner::panMovingCallback, this)),
	subImageReceived(nh.subscribe<std_msgs::Bool>("headSpinner/imageReceived", 1, &HeadSpinner::takenImageCallback, this)),
	pubGetImage(nh.advertise<std_msgs::Bool>("headSpinner/getImage", 1))
{
    d_nh = nh;
    tiltPos.data = degreeToRad(90.0); // camera looking upwards
    panPos.data = degreeToRad(0.0); // camera looking forwards

	ros::Rate r(1);
    r.sleep(); // making sure all subs and pubs are started
    // testing
    // moveHead();
    moveHeadToStart();
    d_panIsMoving = false;
    d_imageTaken = false;
}

void HeadSpinner::moveHeadToStart()
{
	setSpeed(0.8);
	float startPos = -90.0f;
	panPos.data = degreeToRad(startPos);

	tiltPub.publish(tiltPos);
	panPub.publish(panPos);
	d_panIsMoving = true;

	ros::Rate r(10);

	// moving to begin position
	while(d_panIsMoving)
	{
		r.sleep();
		ros::spinOnce();
	}
	setSpeed(2.5);

}

void HeadSpinner::setSpeed(float speed)
{
	 tiltSpeed.request.speed = speed;
	 panSpeed.request.speed = speed;

	 tiltClient.call(tiltSpeed);
	 panClient.call(panSpeed);
}

double HeadSpinner::degreeToRad(double deg)
{
	return deg * M_PI / 180.0;
}

void HeadSpinner::moveHead()
{
	float startPos = -90.0f;
	ros::Rate r(10);

	float stepDegree = 4.0f;
	float currentPos = startPos;
	float maxPos = 90.0f;

	for (float p = currentPos; p <= maxPos; p += stepDegree)
	{
		panPos.data = degreeToRad(p);
		ROS_INFO_STREAM(p);
		panPub.publish(panPos);
		d_panIsMoving = true;

		while (d_panIsMoving)
		{
			r.sleep();
			ros::spinOnce();
		}

		//stopped moving, take image
        std_msgs::Bool t;
        t.data = true;
		pubGetImage.publish(t);
		// wait for image to be taken
		while (!d_imageTaken)
		{
			r.sleep();
			ros::spinOnce();
		}
		d_imageTaken = false;

	}

	moveHeadToStart();

	ROS_INFO_STREAM("Done movement");

}

void HeadSpinner::startCallback(const std_msgs::BoolConstPtr &start)
{
	moveHead();
}

void HeadSpinner::takenImageCallback(const std_msgs::BoolConstPtr &received)
{
	d_imageTaken = true;
}

void HeadSpinner::panMovingCallback(const dynamixel_msgs::JointStateConstPtr &state)
{
	if (state->is_moving)
		d_panIsMoving = true;
	else
		d_panIsMoving = false;

}



