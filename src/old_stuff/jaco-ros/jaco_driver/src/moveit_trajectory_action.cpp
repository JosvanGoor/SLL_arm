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
 *  File: jaco_angles_action.cpp
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

#include "jaco_driver/moveit_trajectory_action.h"
#include <jaco_driver/KinovaTypes.h>
#include "jaco_driver/jaco_types.h"

#define TO_RAD (3.1415 / 180.0)

namespace jaco
{

MoveitTrajectorySubscriber::MoveitTrajectorySubscriber(JacoComm &arm_comm, ros::NodeHandle &n) : 
    arm(arm_comm), 
    trajectory_sub(n.subscribe("moveit_trajectory", 100, &MoveitTrajectorySubscriber::ActionCallback, this))
{}

MoveitTrajectorySubscriber::~MoveitTrajectorySubscriber()
{

}

void MoveitTrajectorySubscriber::ActionCallback(const moveit_msgs::RobotTrajectoryConstPtr &goal)
{
	ROS_INFO("Got an angular goal for the arm");
    
	JacoAngles curPosition;		//holds the current position of the arm

	if (arm.Stopped())
	{
        ROS_ERROR("Arm is stopped, cannot execute trajectory");
		return;
	}

	ros::Rate r(10);
 
    trajectory_msgs::JointTrajectory trajectory = goal->joint_trajectory;

    std::map<std::string, size_t> joints;
    for (size_t j = 0; j < trajectory.joint_names.size(); ++j)
        joints[trajectory.joint_names[j]] = j;

    for (size_t p = 0; p < trajectory.points.size(); ++p)
    {
        trajectory_msgs::JointTrajectoryPoint point = trajectory.points[p];

        // Set up current target and convert from radians (ROS) to degrees (Mico)
        JacoAngles curTarget;
        curTarget.Actuator1 = point.positions[joints["mico_shoulder_yaw_joint"]] / TO_RAD;
        curTarget.Actuator2 = point.positions[joints["mico_shoulder_pitch_joint"]] / TO_RAD;
        curTarget.Actuator3 = point.positions[joints["mico_elbow_pitch_joint"]] / TO_RAD;
        curTarget.Actuator4 = point.positions[joints["mico_elbow_roll_joint"]] / TO_RAD;
        curTarget.Actuator5 = point.positions[joints["mico_wrist_roll_joint"]] / TO_RAD;
        curTarget.Actuator6 = point.positions[joints["mico_hand_roll_joint"]] / TO_RAD;

        AngularInfo velocity;
        velocity.Actuator1 = point.velocities[joints["mico_shoulder_yaw_joint"]] / TO_RAD;
        velocity.Actuator2 = point.velocities[joints["mico_shoulder_pitch_joint"]] / TO_RAD;
        velocity.Actuator3 = point.velocities[joints["mico_elbow_pitch_joint"]] / TO_RAD;
        velocity.Actuator4 = point.velocities[joints["mico_elbow_roll_joint"]] / TO_RAD;
        velocity.Actuator5 = point.velocities[joints["mico_wrist_roll_joint"]] / TO_RAD;
        velocity.Actuator6 = point.velocities[joints["mico_hand_roll_joint"]] / TO_RAD;

        // Start executing the goal
        arm.SetVelocities(velocity);
        arm.SetAngles(curTarget);

        while (true)
        {
            ros::spinOnce();

            // Get current position
            arm.GetAngles(curPosition);

            // Update current postion for feedback
            float positions[6];
            positions[joints["mico_shoulder_yaw_joint"]] = curPosition.Actuator1 * TO_RAD;
            positions[joints["mico_shoulder_pitch_joint"]] = curPosition.Actuator2 * TO_RAD;
            positions[joints["mico_elbow_pitch_joint"]] = curPosition.Actuator3 * TO_RAD;
            positions[joints["mico_elbow_roll_joint"]] = curPosition.Actuator4 * TO_RAD;
            positions[joints["mico_wrist_roll_joint"]] = curPosition.Actuator5 * TO_RAD;
            positions[joints["mico_hand_roll_joint"]] = curPosition.Actuator6 * TO_RAD;
            
            // Calculate error
            bool reached = true;
            for (size_t j = 0; j < 6; ++j)
            {
                float error = positions[j] - point.positions[j];

                float tolerance = 2.0 * TO_RAD;

                if (error > tolerance)
                    reached = false;
            }

            // If the arm stopped, the operation failed
            if (arm.Stopped())
            {
                ROS_ERROR("Arm has stopped. Aborting goal");
                return;
            }

            // If the goal has been reached, continue to the next trajectory point
            if (reached)
            {
                ROS_INFO("TrajectoryPoint reached");
                break;
            }

            r.sleep();
        }
        // Done, proceeding to the next trajectory point
    }

    // Goal reached
    ROS_INFO("Reached goal");
}

} // Namespace jaco
