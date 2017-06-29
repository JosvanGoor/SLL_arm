#include <micoArm.h>

void* checkApiInit(void * usbLib, const char* name)
{
    void * function_pointer = dlsym(usbLib, name);
    assert(function_pointer != NULL);
    return function_pointer;
}

MicoArm::MicoArm(ros::NodeHandle nh) :
		micoTrajectoryServer(nh, "/mico/mico_arm_controller/follow_joint_trajectory", boost::bind(&MicoArm::actionCallback, this, _1), false)
{
	commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so", RTLD_NOW|RTLD_GLOBAL);

	if (commandLayer_handle == NULL)
	{
		ROS_INFO_STREAM("%s" <<  dlerror());
	}

	initAPI = (int (*)())checkApiInit(commandLayer_handle, "InitAPI");
	closeAPI = (int (*)())checkApiInit(commandLayer_handle, "CloseAPI");
	moveHome = (int (*)())checkApiInit(commandLayer_handle, "MoveHome");
	initFingers = (int (*)())checkApiInit(commandLayer_handle,"InitFingers");
	sendAdvanceTrajectory = (int (*)(TrajectoryPoint))checkApiInit(commandLayer_handle, "SendAdvanceTrajectory");
	getAngularCommand = (int (*)(AngularPosition &))checkApiInit(commandLayer_handle, "GetAngularCommand");
	getCartesianPosition = (int (*)(CartesianPosition &))checkApiInit(commandLayer_handle, "GetCartesianPosition");

	ROS_INFO_STREAM("Starting trajectory server.");
	micoTrajectoryServer.start();
	ROS_INFO_STREAM("Trajectory server started.");

	joint_state_publisher = nh.advertise<sensor_msgs::JointState>("/mico/joint_states", 2);

	//timer for publishing joint_states
	status_timer = nh.createTimer(ros::Duration(0.1), &MicoArm::statusTimer, this);

	sendUpRight();
}

MicoArm::~MicoArm()
{
	ROS_INFO_STREAM("Deconstructor called");
	closeAPI();

}

float MicoArm::toRadian(float x)
{
	return x * M_PI / 180.0;
}

void MicoArm::publishJointAngles()
{

	AngularPosition currentAngles;
	getRealJointPositions(currentAngles);

	sensor_msgs::JointState jointState;

	std::vector<std::string> vJointNames;
	vJointNames.push_back("mico_joint_1");
	vJointNames.push_back("mico_joint_2");
	vJointNames.push_back("mico_joint_3");
	vJointNames.push_back("mico_joint_4");
	vJointNames.push_back("mico_joint_5");
	vJointNames.push_back("mico_joint_6");
	vJointNames.push_back("mico_joint_finger_1");
	vJointNames.push_back("mico_joint_finger_2");


	jointState.name = vJointNames;

	jointState.position.resize(8);
	jointState.position[0] = toRadian(currentAngles.Actuators.Actuator1);
	jointState.position[1] = toRadian(currentAngles.Actuators.Actuator2);
	jointState.position[2] = toRadian(currentAngles.Actuators.Actuator3);
	jointState.position[3] = toRadian(currentAngles.Actuators.Actuator4);
	jointState.position[4] = toRadian(currentAngles.Actuators.Actuator5);
	jointState.position[5] = toRadian(currentAngles.Actuators.Actuator6);
	jointState.position[6] = 0.0;
	jointState.position[7] = 0.0;

	joint_state_publisher.publish(jointState);
}

void MicoArm::statusTimer(const ros::TimerEvent&)
{
	publishJointAngles();
}

void MicoArm::sendUpRight()
{
	initAPI();
	ROS_INFO_STREAM("Init to home manually please.");
//	moveHome();
//	initFingers();

	TrajectoryPoint point;
	point.InitStruct();

	point.Position.Type = ANGULAR_POSITION;
	point.LimitationsActive = 1;
	point.Limitations.speedParameter1 = 20.0;
	point.Limitations.speedParameter2 = 20.0;
	point.Position.Actuators.Actuator1 = 0.0;
	point.Position.Actuators.Actuator2 = 180.0;
	point.Position.Actuators.Actuator3 = 180.0;
	point.Position.Actuators.Actuator4 = 0.0;
	point.Position.Actuators.Actuator5 = 0.0;
	point.Position.Actuators.Actuator6 = 0.0;


	sendAdvanceTrajectory(point);
}

void MicoArm::sendVelocity(float speed1, float speed2, float speed3, float speed4, float speed5, float speed6)
{
	TrajectoryPoint point;
	point.InitStruct();

	point.Position.Type = ANGULAR_VELOCITY;
	point.Position.Actuators.Actuator1 = speed1;
	point.Position.Actuators.Actuator2 = speed2;
	point.Position.Actuators.Actuator3 = speed3;
	point.Position.Actuators.Actuator4 = speed4;
	point.Position.Actuators.Actuator5 = speed5;
	point.Position.Actuators.Actuator6 = speed6;

	sendAdvanceTrajectory(point);
}

void MicoArm::limitJoint(float &joint)
{
	joint = joint >= 360 ? fmod(joint, 360) : (joint < 0.0 ? 360 + fmod(joint,360) : joint);
}

void MicoArm::getJointPositions(AngularPosition &joints)
{
	getAngularCommand(joints);

	// Limit the joints between 0 - 360, Except joint 2 and 3, they are limit already
	limitJoint(joints.Actuators.Actuator1);
	limitJoint(joints.Actuators.Actuator4);
	limitJoint(joints.Actuators.Actuator5);
	limitJoint(joints.Actuators.Actuator6);
}

void MicoArm::getRealJointPositions(AngularPosition &joints)
{
	getAngularCommand(joints); // no joint limitations, so joints can be < 0.0 and > 360.0
}

void MicoArm::getEEFPosition(CartesianPosition &eef)
{
	getCartesianPosition(eef);
}


