/**
 *      _____
 *     /  _  \
 *    / _/ \  \
 *   / / \_/   \
 *  /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
 *  \  / \_/ \  / /  _\| |  | __| / _ \ | ++ \ | ++ \ / _ \ |_   _|| | | |
 *   \ \_/ \_/ /  | |  | |  | ++ | |_| || ++ / | ++_/| |_| |  | |  | +-+ |
 *    \  \_/  /   | |_ | |_ | ++ |  _  || |\ \ | |   |  _  |  | |  | +-+ |
 *     \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
 *             ROBOTICS™
 *
 *  File: jaco_comm.cpp
 *  Desc: Class for moving/querying jaco arm.
 *  Auth: Alex Bencz, Jeff Schmidt
 *
 *  Copyright (c) 2013, Clearpath Robotics, Inc.
 *  All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to skynet@clearpathrobotics.com
 *
 */

#include <ros/ros.h>
#include "jaco_driver/jaco_comm.h"
#include <string>
#include <vector>
 #include <math.h>


namespace jaco
{

JacoComm::JacoComm(const ros::NodeHandle& node_handle,
                   boost::recursive_mutex &api_mutex,
                   const bool is_movement_on_start)
    : is_software_stop_(false), api_mutex_(api_mutex)
{
	vLastCall.resize(1);
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    // Get the serial number parameter for the arm we wish to connec to
    std::string serial_number = "";
    //node_handle.getParam("serial_number", serial_number);

    std::vector<int> api_version;
    int result = jaco_api_.getAPIVersion(api_version);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not get the Kinova API version", result);
    }

    ROS_INFO_STREAM("Initializing Kinova API (header version: " << COMMAND_LAYER_VERSION << ", library version: "
                    << api_version[0] << "." << api_version[1] << "." << api_version[2] << ")");

    result = jaco_api_.initAPI();
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not initialize Kinova API", result);
    }

    std::vector<KinovaDevice> devices_list;
    result = NO_ERROR_KINOVA;
    jaco_api_.getDevices(devices_list, result);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not get devices list", result);
    }

    bool found_arm = false;
    for (int device_i = 0; device_i < devices_list.size(); device_i++)
    {
        // If no device is specified, just use the first available device
        if ((serial_number == "")
            || (std::strcmp(serial_number.c_str(), devices_list[device_i].SerialNumber) == 0))
        {
            result = jaco_api_.setActiveDevice(devices_list[device_i]);
            if (result != NO_ERROR_KINOVA)
            {
                throw JacoCommException("Could not set the active device", result);
            }

            GeneralInformations general_info;
            result = jaco_api_.getGeneralInformations(general_info);
            if (result != NO_ERROR_KINOVA)
            {
                throw JacoCommException("Could not get general information about the device", result);
            }

            ClientConfigurations configuration;
            getConfig(configuration);

            QuickStatus quick_status;
            getQuickStatus(quick_status);

            if ((quick_status.RobotType != 0) && (quick_status.RobotType != 1))
            {
                ROS_ERROR("Could not get the type of the arm from the quick status, expected "
                          "either type 0 (JACO), or type 1 (MICO), got %d", quick_status.RobotType);
                throw JacoCommException("Could not get the type of the arm", quick_status.RobotType);
            }

            num_fingers_ = quick_status.RobotType == 0 ? 3 : 2;

            ROS_INFO_STREAM("Found " << devices_list.size() << " device(s), using device at index " << device_i
                            << " (model: " << configuration.Model
                            << ", serial number: " << devices_list[device_i].SerialNumber
                            << ", code version: " << general_info.CodeVersion
                            << ", code revision: " << general_info.CodeRevision << ")");

            found_arm = true;
            break;
        }
    }

    if (!found_arm)
    {
        ROS_ERROR("Could not find the specified arm (serial: %s) among the %d attached devices",
                  serial_number.c_str(), static_cast<int>(devices_list.size()));
        throw JacoCommException("Could not find the specified arm", 0);
    }

    // On a cold boot the arm may not respond to commands from the API right away.
    // This kick-starts the Control API so that it's ready to go.
    startAPI();
 //   stopAPI();
 //   startAPI();

    // Set the angular velocity of each of the joints to zero
    TrajectoryPoint jaco_velocity;
    memset(&jaco_velocity, 0, sizeof(jaco_velocity));
    setCartesianVelocities(jaco_velocity.Position.CartesianPosition);

    if (is_movement_on_start)
    {
        initFingers();
    }
    else
    {
        ROS_WARN("Movement on connection to the arm has been suppressed on initialization. You may "
                 "have to home the arm (through the home service) before movement is possible");
    }
}


JacoComm::~JacoComm()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    jaco_api_.closeAPI();
}


/*!
 * \brief Determines whether the arm has returned to its "Home" state.
 *
 * Checks the current joint angles, then compares them to the known "Home"
 * joint angles.
 */
bool JacoComm::isHomed(void)
{
    QuickStatus quick_status;
    getQuickStatus(quick_status);

    if (quick_status.RetractType == 1)
    {
        return true;
    }
    else
    {
        return false;
    }
}


/*!
 * \brief Send the arm to the "home" position.
 *
 * The code replicates the function of the "home" button on the user controller
 * by "pressing" the home button long enough for the arm to return to the home
 * position.
 *
 * Fingers are homed by manually opening them fully, then returning them to a
 * half-open position.
 */
void JacoComm::homeArm(void)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_INFO("Arm is stopped, cannot home");
        return;
    }
    else if (isHomed())
    {
        ROS_INFO("Arm is already in \"home\" position");
        return;
    }

    stopAPI();
    ros::Duration(1.0).sleep();
    startAPI();

    ROS_INFO("Homing the arm");
    int result = jaco_api_.moveHome();
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Move home failed", result);
    }
}


/*!
 * \brief Initialize finger actuators.
 *
 * Move fingers to the full-open position to initialize them for use.
 * Note, The this routine requires firmware version 5.05.x (or higher?).
 */
void JacoComm::initFingers(void)
{
    ROS_INFO("Initializing fingers...this will take a few seconds and the fingers should open completely");
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int result = jaco_api_.initFingers();
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not init fingers", result);
    }
    return;
}

void JacoComm::setTrajectory(const JacoAngles &angles)
{
	boost::recursive_mutex::scoped_lock lock(api_mutex_);
	if (isStopped())
		return;

	ROS_INFO_STREAM("SetTrajectory....");
	jaco_api_.setAngularControl();

	TrajectoryPoint tp;
	tp.InitStruct();

	tp.Limitations.speedParameter1 = 10.0f;
	tp.Limitations.speedParameter2 = 20.0f;
	tp.Limitations.speedParameter3 = 20.0f;

	tp.LimitationsActive = true;
	tp.Position.Delay = 0.0;
	tp.Position.HandMode = HAND_NOMOVEMENT;
	tp.Position.Type = ANGULAR_POSITION;

    jaco_api_.eraseAllTrajectories(); // else it will stutter like crazy between each point
    tp.Position.Actuators.Actuator1 = (float)angles.Actuator1;
    tp.Position.Actuators.Actuator2 = (float)angles.Actuator2;
    tp.Position.Actuators.Actuator3 = (float)angles.Actuator3;
    tp.Position.Actuators.Actuator4 = (float)angles.Actuator4;
    tp.Position.Actuators.Actuator5 = (float)angles.Actuator5;
    tp.Position.Actuators.Actuator6 = (float)angles.Actuator6;
    tp.Position.Actuators.Actuator1 -= (float)20.0f; // For the offset in the hardware
    jaco_api_.sendAdvanceTrajectory(tp);
}


/*!
 * \brief Sends a joint angle command to the Jaco arm.
 *
 * Waits until the arm has stopped moving before releasing control of the API.
 */
void JacoComm::setJointAngles(const JacoAngles &angles, int timeout, bool push)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_INFO("The angles could not be set because the arm is stopped");
        return;
    }

    int result = NO_ERROR_KINOVA;
    TrajectoryPoint jaco_position;
    jaco_position.InitStruct();
    memset(&jaco_position, 0, sizeof(jaco_position));  // zero structure

    if (push)
    {
        result = jaco_api_.eraseAllTrajectories();
        if (result != NO_ERROR_KINOVA)
        {
            throw JacoCommException("Could not erase trajectories", result);
        }
    }

    startAPI();

    result = jaco_api_.setAngularControl();
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not set angular control", result);
    }

    jaco_position.Position.Delay = 0.0;
    jaco_position.Position.Type = ANGULAR_POSITION;
    jaco_position.Position.Actuators = angles;

    result = jaco_api_.sendAdvanceTrajectory(jaco_position);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not send advanced joint angle trajectory", result);
    }
}

bool checkFreeze(JacoPose &position, JacoPose &prevPosition)
{
    if (position.X == prevPosition.X &&
        position.Y == prevPosition.Y &&
        position.Z == prevPosition.Z &&
        position.ThetaX == prevPosition.ThetaX &&
        position.ThetaY == prevPosition.ThetaY &&
        position.ThetaZ == prevPosition.ThetaZ)
        return true; // most likely the arm is frozen in place 
    else
        return false;
}

bool JacoComm::sendCartesianPosition(const JacoPose &position, bool approach)
{
	LastCall lastCall;
	lastCall.function = "Send";
	lastCall.position = position;
	lastCall.approach = approach;

	vLastCall.at(0) = lastCall;

	boost::recursive_mutex::scoped_lock lock(api_mutex_);

	if (isStopped())
	{
		ROS_INFO_STREAM("The position could not be set because the arm is stopped");
		return false;
	}

	int result = NO_ERROR_KINOVA;

	TrajectoryPoint micoPosition;
	micoPosition.InitStruct();

	micoPosition.Position.Delay = 0.0;
	micoPosition.Position.Type = CARTESIAN_POSITION;
	micoPosition.Position.HandMode = HAND_NOMOVEMENT;

	result = jaco_api_.eraseAllTrajectories(); // The trajectory buffer is shit!
	if (result != NO_ERROR_KINOVA)
	{
		throw JacoCommException("Could not erase trajectories", result);
	}

	if (approach)
	{
		micoPosition.Limitations.speedParameter1 = 0.05f;
		micoPosition.Limitations.speedParameter2 = 0.25f;
		micoPosition.LimitationsActive = true;
	}
	else
	{
		micoPosition.Limitations.speedParameter1 = 0.10f;
		micoPosition.Limitations.speedParameter2 = 0.45f;
		micoPosition.LimitationsActive = false;
	}



	// These values will not be used but are initialized anyway.
	micoPosition.Position.Actuators.Actuator1 = 0.0f;
	micoPosition.Position.Actuators.Actuator2 = 0.0f;
	micoPosition.Position.Actuators.Actuator3 = 0.0f;
	micoPosition.Position.Actuators.Actuator4 = 0.0f;
	micoPosition.Position.Actuators.Actuator5 = 0.0f;
	micoPosition.Position.Actuators.Actuator6 = 0.0f;

	micoPosition.Position.CartesianPosition = position;

	result = jaco_api_.sendAdvanceTrajectory(micoPosition);

	if (result != NO_ERROR_KINOVA)
	{
		throw JacoCommException("Could not send basic trajectory", result);
	}
}

/*!
 * \brief Sends a cartesian coordinate trajectory to the Jaco arm.
 *
 * Waits until the arm has stopped moving before releasing control of the API.
 */
bool JacoComm::setCartesianPosition(const JacoPose &position, int timeout, bool push, float speed1, float speed2)
{
	LastCall lastCall;
	lastCall.function = "Set";
	lastCall.position = position;
	lastCall.timeout = timeout;
	lastCall.push = push;
	lastCall.speed1 = speed1;
	lastCall.speed2 = speed2;

	vLastCall.at(0) = lastCall;

    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    bool succesfull = false;

    if (isStopped())
    {
        ROS_INFO("The position could not be set because the arm is stopped");
        return false;
    }

    int result = NO_ERROR_KINOVA;
    TrajectoryPoint jaco_position;
    jaco_position.InitStruct();
    memset(&jaco_position, 0, sizeof(jaco_position));  // zero structure

    if (push)
    {
        result = jaco_api_.eraseAllTrajectories();
        if (result != NO_ERROR_KINOVA)
        {
            throw JacoCommException("Could not erase trajectories", result);
        }
    }

    jaco_position.Position.Delay = 0.0;
    jaco_position.Position.Type = CARTESIAN_POSITION;
    jaco_position.Position.HandMode = HAND_NOMOVEMENT;
    jaco_position.Limitations.speedParameter1 = speed1;
    jaco_position.Limitations.speedParameter2 = speed2;
    jaco_position.LimitationsActive = true;

    // These values will not be used but are initialized anyway.
    jaco_position.Position.Actuators.Actuator1 = 0.0f;
    jaco_position.Position.Actuators.Actuator2 = 0.0f;
    jaco_position.Position.Actuators.Actuator3 = 0.0f;
    jaco_position.Position.Actuators.Actuator4 = 0.0f;
    jaco_position.Position.Actuators.Actuator5 = 0.0f;
    jaco_position.Position.Actuators.Actuator6 = 0.0f;

    jaco_position.Position.CartesianPosition = position;

    result = jaco_api_.sendAdvanceTrajectory(jaco_position);

    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not send basic trajectory", result);
    }

    bool bError = true;
    float errorRating = 0.03f;
    float errorRot = 10.0f;
    JacoPose positionn;
    JacoPose prevPosition;

    int freezeCounter = 0;
    int const freezeCounterMax = 20;

    ros::Rate loop_rate(10);

    float prevX, prevY, prevZ, prevThetaX, prevThetaY, prevThetaZ;

    getCartesianPosition(prevPosition);

    loop_rate.sleep();

    while(bError == true)
    {
        getCartesianPosition(positionn);
         
        float positionXDeg = positionn.ThetaX * (180.0f / M_PI); 
        float positionYDeg = positionn.ThetaY * (180.0f / M_PI); 
        float positionZDeg = positionn.ThetaZ * (180.0f / M_PI); 

        float positionTargetXDeg = jaco_position.Position.CartesianPosition.ThetaX * (180.0f / M_PI); 
        float positionTargetYDeg = jaco_position.Position.CartesianPosition.ThetaY * (180.0f / M_PI); 
        float positionTargetZDeg = jaco_position.Position.CartesianPosition.ThetaZ * (180.0f / M_PI); 


        ROS_INFO_STREAM("Position rotation: " << positionXDeg << ", " << positionYDeg << ", " << positionZDeg);
        ROS_INFO_STREAM("Target rotation: " << positionTargetXDeg << ", " << positionTargetYDeg << ", " << positionTargetZDeg);

        float diffX = fabs(positionXDeg - positionTargetXDeg);
        
        if (diffX > 180.0f)
            diffX = 360.0f - diffX; 

        float diffY = fabs(positionYDeg - positionTargetYDeg);
        
        if (diffY > 180.0f)
            diffY = 360.0f - diffY; 

        float diffZ = fabs(positionZDeg - positionTargetZDeg);
        
        if (diffZ > 180.0f)
            diffZ = 360.0f - diffZ; 

        ROS_INFO_STREAM("DIFF: " << diffX << ", " << diffY << ", " << diffZ);

        if (fabs(positionn.Z - jaco_position.Position.CartesianPosition.Z) <= errorRating &&
            fabs(positionn.Y - jaco_position.Position.CartesianPosition.Y) <= errorRating &&
            fabs(positionn.X - jaco_position.Position.CartesianPosition.X) <= errorRating &&
            diffX <= errorRot &&
            diffY <= errorRot &&
            diffY <= errorRot)
            bError = false;

        if (checkFreeze(positionn, prevPosition))
            ++freezeCounter;

        if (freezeCounter == freezeCounterMax)
        {
            result = jaco_api_.eraseAllTrajectories();

            if (result != NO_ERROR_KINOVA)
                throw JacoCommException("Could not erase trajectories", result);

            return false; // unable to execute, the arm froze in position
        }

        prevPosition = positionn;

        loop_rate.sleep();
/*
        ROS_INFO_STREAM("Currenct X-Position: " << positionn.X << ", Diff: "  << fabs(positionn.X - jaco_position.Position.CartesianPosition.X) );
        ROS_INFO_STREAM("Currenct Y-Position: " << positionn.Y << ", Diff: "  << fabs(positionn.Y - jaco_position.Position.CartesianPosition.Y) );
        ROS_INFO_STREAM("Currenct Z-Position: " << positionn.Z << ", Diff: "  << fabs(positionn.Z - jaco_position.Position.CartesianPosition.Z) );
        ROS_INFO_STREAM("Currenct X-Theta: " << positionn.ThetaX << ", Diff: "  << fabs(positionn.ThetaX - jaco_position.Position.CartesianPosition.ThetaX) );
        ROS_INFO_STREAM("Currenct Y-Theta: " << positionn.ThetaY << ", Diff: "  << fabs(positionn.ThetaY - jaco_position.Position.CartesianPosition.ThetaY) );
        ROS_INFO_STREAM("Currenct Z-Theta: " << positionn.ThetaZ << ", Diff: "  << fabs(positionn.ThetaZ - jaco_position.Position.CartesianPosition.ThetaZ) );
*/
    }

    succesfull = true; // everything went good
    return succesfull;
}


/*!
 * \brief Sets the finger positions
 */
void JacoComm::setFingerPositions(const FingerAngles &fingers, int timeout, bool push)
{
	LastCall lastCall;
	lastCall.function = "fingers";
	lastCall.fingers = fingers;
	lastCall.timeout = timeout;
	lastCall.push = push;
	vLastCall.push_back(lastCall);

    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_INFO("The fingers could not be set because the arm is stopped");
        return;
    }

    int result = NO_ERROR_KINOVA;
    TrajectoryPoint jaco_position;
    jaco_position.InitStruct();
    memset(&jaco_position, 0, sizeof(jaco_position));  // zero structure

    if (push)
    {
        result = jaco_api_.eraseAllTrajectories();
        if (result != NO_ERROR_KINOVA)
        {
            throw JacoCommException("Could not erase trajectories", result);
        }
    }

  //  startAPI();

  /*  result = jaco_api_.setCartesianControl();
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not set Cartesian control", result);
    }
 */
    // Initialize Cartesian control of the fingers
    jaco_position.Position.HandMode = POSITION_MODE;
    jaco_position.Position.Type = CARTESIAN_POSITION;
    jaco_position.Position.Fingers = fingers;
    jaco_position.Limitations.speedParameter1 = 0.01;
    jaco_position.Limitations.speedParameter2 = 0.01;
    jaco_position.Limitations.speedParameter3 = 0.01;
    jaco_position.LimitationsActive = true;
    jaco_position.Position.Delay = 0.0;

    // These values will not be used but are initialized anyway.
    JacoAngles angles;
    getJointAngles(angles);
    jaco_position.Position.Actuators = angles;

    // When loading a cartesian position for the fingers, values are required for the arm joints
    // as well or the arm goes nuts.  Grab the current position and feed it back to the arm.
    JacoPose pose;
    getCartesianPosition(pose);
    jaco_position.Position.CartesianPosition = pose;

    result = jaco_api_.sendAdvanceTrajectory(jaco_position);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not send advanced finger trajectory", result);
    }
}


/*!
 * \brief Set the angular velocity of the joints
 */
void JacoComm::setJointVelocities(const AngularInfo &joint_vel)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_INFO("The velocities could not be set because the arm is stopped");
        return;
    }

    TrajectoryPoint jaco_velocity;
    jaco_velocity.InitStruct();

    memset(&jaco_velocity, 0, sizeof(jaco_velocity));  // zero structure

    startAPI();
    jaco_velocity.Position.Type = ANGULAR_VELOCITY;

    // confusingly, velocity is passed in the position struct
    jaco_velocity.Position.Actuators = joint_vel;

    int result = jaco_api_.sendAdvanceTrajectory(jaco_velocity);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not send advanced joint velocity trajectory", result);
    }
}


/*!
 * \brief Set the cartesian velocity of the tool tip
 */
void JacoComm::setCartesianVelocities(const CartesianInfo &velocities)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);

    if (isStopped())
    {
        ROS_INFO("The cartesian velocities could not be set because the arm is stopped");
        jaco_api_.eraseAllTrajectories();
        return;
    }

    TrajectoryPoint jaco_velocity;
    jaco_velocity.InitStruct();

    memset(&jaco_velocity, 0, sizeof(jaco_velocity));  // zero structure

    startAPI();
    jaco_velocity.Position.Type = CARTESIAN_VELOCITY;

    // confusingly, velocity is passed in the position struct
    jaco_velocity.Position.CartesianPosition = velocities;

    int result = jaco_api_.sendAdvanceTrajectory(jaco_velocity);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not send advanced Cartesian velocity trajectory", result);
    }
}


/*!
 * \brief Obtains the current arm configuration.
 *
 * This is the configuration which are stored on the arm itself. Many of these
 * configurations may be set using the Windows interface.
 */
void JacoComm::setConfig(const ClientConfigurations &config)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    int result = jaco_api_.setClientConfigurations(config);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not set the client configuration", result);
    }
}


/*!
 * \brief API call to obtain the current angular position of all the joints.
 */
void JacoComm::getJointAngles(JacoAngles &angles)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    AngularPosition jaco_angles;
    memset(&jaco_angles, 0, sizeof(jaco_angles));  // zero structure

    int result = jaco_api_.getAngularPosition(jaco_angles);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not get the angular position", result);
    }
    
//    jaco_angles.Actuators.Actuator1 += 20; // For the offset in the hardware
    angles = JacoAngles(jaco_angles.Actuators);
}

/*!
 * \brief API call to obtain the correct current angular position of all the joints.
 */
void JacoComm::getCorrectJointAngles(JacoAngles &angles)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    AngularPosition jaco_angles;
    memset(&jaco_angles, 0, sizeof(jaco_angles));  // zero structure

    int result = jaco_api_.getAngularPosition(jaco_angles);
    
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not get the angular position", result);
    }
    
//	jaco_angles.Actuators.Actuator1 += 20; // For the offset in the hardware

	// gives back the actual angle position of the arm!
	angles.Actuator1 = jaco_angles.Actuators.Actuator1;
	angles.Actuator2 = jaco_angles.Actuators.Actuator2;
	angles.Actuator3 = jaco_angles.Actuators.Actuator3;
	angles.Actuator4 = jaco_angles.Actuators.Actuator4;
	angles.Actuator5 = jaco_angles.Actuators.Actuator5;
	angles.Actuator6 = jaco_angles.Actuators.Actuator6;
 }


/*!
 * \brief API call to obtain the current cartesian position of the arm.
 */
void JacoComm::getCartesianPosition(JacoPose &position)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    CartesianPosition jaco_cartesian_position;
    memset(&jaco_cartesian_position, 0, sizeof(jaco_cartesian_position));  // zero structure

    isStopped();
    int result = jaco_api_.getCartesianPosition(jaco_cartesian_position);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not get the Cartesian position", result);
    }

    position = JacoPose(jaco_cartesian_position.Coordinates);
}

void JacoComm::getCartesianForce(CartesianPosition &position)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    CartesianPosition jaco_cartesian_position;
    memset(&jaco_cartesian_position, 0, sizeof(jaco_cartesian_position));  // zero structure

    int result = jaco_api_.getCartesianForce(jaco_cartesian_position);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not get the Cartesian position", result);
    }

    position = jaco_cartesian_position;
}

void JacoComm::getAngularCurrent(AngularPosition &position)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    AngularPosition jaco_cartesian_position;
    memset(&jaco_cartesian_position, 0, sizeof(jaco_cartesian_position));  // zero structure

    int result = jaco_api_.getAngularCurrent(jaco_cartesian_position);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not get the Cartesian position", result);
    }

    position = jaco_cartesian_position;
}


/*!
 * \brief API call to obtain the current finger positions.
 */
void JacoComm::getFingerPositions(FingerAngles &fingers)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    CartesianPosition jaco_cartesian_position;
    memset(&jaco_cartesian_position, 0, sizeof(jaco_cartesian_position));  // zero structure

    isStopped();

    int result = jaco_api_.getCartesianPosition(jaco_cartesian_position);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not get Cartesian finger position", result);
    }

    if (num_fingers_ == 2)
    {
        jaco_cartesian_position.Fingers.Finger3 = 0.0;
    }

    fingers = FingerAngles(jaco_cartesian_position.Fingers);
}


/*!
 * \brief API call to obtain the current client configuration.
 */
void JacoComm::getConfig(ClientConfigurations &config)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&config, 0, sizeof(config));  // zero structure

    int result = jaco_api_.getClientConfigurations(config);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not get client configuration", result);
    }
}


/*!
 * \brief API call to obtain the current "quick status".
 */
void JacoComm::getQuickStatus(QuickStatus &quick_status)
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    memset(&quick_status, 0, sizeof(quick_status));  // zero structure
    int result = jaco_api_.getQuickStatus(quick_status);
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not get quick status", result);
    }
}


void JacoComm::stopAPI()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    is_software_stop_ = true;

    int result = jaco_api_.stopControlAPI();
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not stop the control API", result);
    }

    result = jaco_api_.eraseAllTrajectories();
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not erase all trajectories", result);
    }
}

void JacoComm::eStart() // start from eButton is received
{
	is_software_stop_ = false;
}

void JacoComm::startAPI()
{
    boost::recursive_mutex::scoped_lock lock(api_mutex_);
    if (isStopped())
    {
        is_software_stop_ = false;
        jaco_api_.stopControlAPI();
        ros::Duration(0.05).sleep();
    }

    jaco_api_.stopControlAPI();
    int result = jaco_api_.startControlAPI();
    if (result != NO_ERROR_KINOVA)
    {
        throw JacoCommException("Could not start the control API", result);
    }
}


int JacoComm::numFingers()
{
    return num_fingers_;
}


/*!
 * \brief Dumps the current joint angles onto the screen.
 */
void JacoComm::printAngles(const JacoAngles &angles)
{
    ROS_INFO("Joint angles (deg) -- J1: %f, J2: %f J3: %f, J4: %f, J5: %f, J6: %f",
             angles.Actuator1, angles.Actuator2, angles.Actuator3,
             angles.Actuator4, angles.Actuator5, angles.Actuator6);
}


/*!
 * \brief Dumps the current cartesian positions onto the screen.
 */
void JacoComm::printPosition(const JacoPose &position)
{
    ROS_INFO("Arm position\n"
             "\tposition (m) -- x: %f, y: %f z: %f\n"
             "\trotation (rad) -- theta_x: %f, theta_y: %f, theta_z: %f",
             position.X, position.Y, position.Z,
             position.ThetaX, position.ThetaY, position.ThetaZ);
}


/*!
 * \brief Dumps the current finger positions onto the screen.
 */
void JacoComm::printFingers(const FingersPosition &fingers)
{
    ROS_INFO("Finger positions -- F1: %f, F2: %f, F3: %f",
             fingers.Finger1, fingers.Finger2, fingers.Finger3);
}


/*!
 * \brief Dumps the client configuration onto the screen.
 */
void JacoComm::printConfig(const ClientConfigurations &config)
{
    ROS_INFO_STREAM("Arm configuration:\n"
                    "\tClientID: " << config.ClientID <<
                    "\n\tClientName: " << config.ClientName <<
                    "\n\tOrganization: " << config.Organization <<
                    "\n\tSerial:" << config.Serial <<
                    "\n\tModel: " << config.Model <<
                    "\n\tMaxForce: " << config.MaxForce <<
                    "\n\tSensibility: " <<  config.Sensibility <<
                    "\n\tDrinkingHeight: " << config.DrinkingHeight <<
                    "\n\tComplexRetractActive: " << config.ComplexRetractActive <<
                    "\n\tRetractedPositionAngle: " << config.RetractedPositionAngle <<
                    "\n\tRetractedPositionCount: " << config.RetractedPositionCount <<
                    "\n\tDrinkingDistance: " << config.DrinkingDistance <<
                    "\n\tFingers2and3Inverted: " << config.Fingers2and3Inverted <<
                    "\n\tDrinkingLength: " << config.DrinkingLenght <<
                    "\n\tDeletePreProgrammedPositionsAtRetract: " <<
                    config.DeletePreProgrammedPositionsAtRetract <<
                    "\n\tEnableFlashErrorLog: " << config.EnableFlashErrorLog <<
                    "\n\tEnableFlashPositionLog: " << config.EnableFlashPositionLog);
}


bool JacoComm::isStopped()
{
	if (is_software_stop_)
	{
		ros::Rate r(2);
		while (is_software_stop_)
		{
			ROS_INFO_STREAM("STOPPED: " << is_software_stop_);
			ros::spinOnce();
			r.sleep();
		}

		// recall the last function
		if (vLastCall.at(0).function == "Send")
			sendCartesianPosition(vLastCall.at(0).position, vLastCall.at(0).approach);
		if (vLastCall.at(0).function == "Set")
			setCartesianPosition(vLastCall.at(0).position, vLastCall.at(0).timeout, vLastCall.at(0).push, vLastCall.at(0).speed1, vLastCall.at(0).speed2);
		if (vLastCall.at(0).function == "fingers")
			setFingerPositions(vLastCall.at(0).fingers, vLastCall.at(0).timeout, vLastCall.at(0).push);
	}

    return is_software_stop_;
}


}  // namespace jaco
