#include <ros/ros.h>
#include <micoArm.h>
#include <signal.h>
#include <vrep_test/GetJointPosition.h>
#include <vrep_test/GetEEFPosition.h>
#include <vrep_test/SetJointVelocity.h>
#include <std_msgs/Float32.h>

bool quit = false; // signal flag.
ros::NodeHandle nh;
MicoArm mico(nh);
float velocity;

void sigintHandler(int sig)
{
	quit = true;
}

void moveJoint(float joint)
{

	AngularPosition joints;
	mico.getJointPositions(joints);

	ROS_INFO_STREAM(joints.Actuators.Actuator1 << ", " << joints.Actuators.Actuator2 << ", " <<
			joints.Actuators.Actuator3 << ", " << joints.Actuators.Actuator4 << ", " <<
			joints.Actuators.Actuator5 << ", " << joints.Actuators.Actuator6);

}


void setJointVelocity(vrep_test::SetJointVelocity velocity)
{
	mico.sendVelocity(velocity.joint1, velocity.joint2, velocity.joint3, velocity.joint4, velocity.joint5, velocity.joint6);
}


/*
bool setJointVelocity(vrep_test::SetJointVelocity::Request &req, vrep_test::SetJointVelocity::Response &res)
{
	for (size_t t = 0; t < 50; ++t)
	{
		mico.sendVelocity(req.joint1, req.joint2, req.joint3, req.joint4, req.joint5, req.joint6);
		usleep(5000);
	}

	return true;
}
*/
bool getJointPosition(vrep_test::GetJointPosition::Request &req, vrep_test::GetJointPosition::Response &res)
{
	//ROS_INFO_STREAM("Getting Joint Positions");
	AngularPosition joints;
	mico.getJointPositions(joints);
	res.joint1 = joints.Actuators.Actuator1;
	res.joint2 = joints.Actuators.Actuator2;
	res.joint3 = joints.Actuators.Actuator3;
	res.joint4 = joints.Actuators.Actuator4;
	res.joint5 = joints.Actuators.Actuator5;
	res.joint6 = joints.Actuators.Actuator6;
	return true;
}

bool getEEFPosition(vrep_test::GetEEFPosition::Request &req, vrep_test::GetEEFPosition::Response &res)
{
	CartesianPosition eef;
	mico.getEEFPosition(eef);
	res.x = eef.Coordinates.X;
	res.y = eef.Coordinates.Y;
	res.z = eef.Coordinates.Z;
	return true;
}


int main(int argc, char **argv)
{
	velocity = 0.0f;
	ros::init(argc, argv, "mico_controller");
	ros::NodeHandle nh;
	signal(SIGINT, sigintHandler);


	mico.sendUpRight(); // init arm to up right position

	ros::ServiceServer jointPositionServer = nh.advertiseService("getJointPosition", getJointPosition);
	ros::ServiceServer eefPositionServer = nh.advertiseService("getEEFPosition", getEEFPosition);
//	ros::ServiceServer jointVelocityServer = nh.advertiseService("setJointVelocity", setJointVelocity);

	ros::Subscriber jointVelocitySubscriber = nh.subscribe("setJointVelocity",1, setJointVelocity);

	ros::Rate r(200);
	while(true)
	{
		ros::spinOnce();
		r.sleep();
		if (quit) break;
	}

}
