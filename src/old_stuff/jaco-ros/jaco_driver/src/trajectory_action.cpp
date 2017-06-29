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

#include "jaco_driver/trajectory_action.h"
#include <kinova/KinovaTypes.h>
#include "jaco_driver/jaco_types.h"

#define TO_RAD (3.14159 / 180.0)
#define TO_DEGREE (180.0f / 3.14159f)
namespace jaco
{

TrajectoryActionServer::TrajectoryActionServer(JacoComm &arm_comm, ros::NodeHandle &n) : 
    arm(arm_comm), 
    as_(n, "trajectory", boost::bind(&TrajectoryActionServer::ActionCallback, this, _1), false)
{
    as_.start();
}

TrajectoryActionServer::~TrajectoryActionServer()
{

}

void TrajectoryActionServer::ActionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{

    control_msgs::FollowJointTrajectoryFeedback feedback;
    control_msgs::FollowJointTrajectoryResult result;

    ROS_INFO("Got an angular goal for the arm");
    
    JacoAngles curPosition;		//holds the current position of the arm

    if (arm.isStopped())
    {
        arm.getJointAngles(curPosition);
        as_.setAborted(result);
        return;
    }

    ros::Rate r(10);
 
    const float tolerance = 2.0; 	//dead zone for position (degrees)

    trajectory_msgs::JointTrajectory trajectory = goal->trajectory;
    feedback.joint_names = trajectory.joint_names;

    std::map<std::string, size_t> joints;
    for (size_t j = 0; j < trajectory.joint_names.size(); ++j)
        joints[trajectory.joint_names[j]] = j;


    trajectory_msgs::JointTrajectoryPoint point = trajectory.points[0];
    jaco_msgs::JointAngles target;
    

    JacoAngles curTarget(target);
    arm.getCorrectJointAngles(curPosition); // get correct

    JacoAngles correctAngles;
    JacoAngles positiveAngles;

    arm.getCorrectJointAngles(correctAngles);  // get correct
    arm.getJointAngles(positiveAngles);

    ROS_INFO_STREAM("DIFFERENCE 1: " << correctAngles.Actuator1 << " - " << positiveAngles.Actuator1 << " = " << correctAngles.Actuator1 - positiveAngles.Actuator1);
    ROS_INFO_STREAM("DIFFERENCE 2: " << correctAngles.Actuator2 << " - " << positiveAngles.Actuator2 << " = " << correctAngles.Actuator2 - positiveAngles.Actuator2);
    ROS_INFO_STREAM("DIFFERENCE 3: " << correctAngles.Actuator3 << " - " << positiveAngles.Actuator3 << " = " << correctAngles.Actuator3 - positiveAngles.Actuator3);
    ROS_INFO_STREAM("DIFFERENCE 4: " << correctAngles.Actuator4 << " - " << positiveAngles.Actuator4 << " = " << correctAngles.Actuator4 - positiveAngles.Actuator4);
    ROS_INFO_STREAM("DIFFERENCE 5: " << correctAngles.Actuator5 << " - " << positiveAngles.Actuator5 << " = " << correctAngles.Actuator5 - positiveAngles.Actuator5);
    ROS_INFO_STREAM("DIFFERENCE 6: " << correctAngles.Actuator6 << " - " << positiveAngles.Actuator6 << " = " << correctAngles.Actuator6 - positiveAngles.Actuator6);

    double actuator1Correction = correctAngles.Actuator1 - positiveAngles.Actuator1;
    double actuator2Correction = correctAngles.Actuator2 - positiveAngles.Actuator2;
    double actuator3Correction = correctAngles.Actuator3 - positiveAngles.Actuator3;
    double actuator4Correction = correctAngles.Actuator4 - positiveAngles.Actuator4;
    double actuator5Correction = correctAngles.Actuator5 - positiveAngles.Actuator5;
    double actuator6Correction = correctAngles.Actuator6 - positiveAngles.Actuator6;


    for (size_t p = 0; p < trajectory.points.size(); ++p) // send all points to the FIFO trajectory buffer on the arm
    {
        trajectory_msgs::JointTrajectoryPoint point = trajectory.points[p];

        //Initializing the position vector size
        feedback.actual.positions.resize(6);
        //Initializing the jacotrajectory points for the feedback
        feedback.desired.positions.resize(6);
        feedback.error.positions.resize(6);
        feedback.desired = trajectory.points[p];

        // Set up current target and convert from radians (ROS) to degrees (Mico)
        //TODO:Fix a constructor that accepts points!
        jaco_msgs::JointAngles target;
        target.joint1 = point.positions[joints["mico_shoulder_yaw_joint"]];
        target.joint2 = point.positions[joints["mico_shoulder_pitch_joint"]];
        target.joint3 = point.positions[joints["mico_elbow_pitch_joint"]];
        target.joint4 = point.positions[joints["mico_elbow_roll_joint"]];
        target.joint5 = point.positions[joints["mico_wrist_roll_joint"]];
        target.joint6 = point.positions[joints["mico_hand_roll_joint"]];
        JacoAngles curTarget(target);

        //TODO:Check whether the velocities are being set correctly
   /*     AngularInfo velocity;
        velocity.Actuator1 = point.velocities[joints["mico_shoulder_yaw_joint"]] / TO_RAD;
        velocity.Actuator2 = point.velocities[joints["mico_shoulder_pitch_joint"]] / TO_RAD;
        velocity.Actuator3 = point.velocities[joints["mico_elbow_pitch_joint"]] / TO_RAD;
        velocity.Actuator4 = point.velocities[joints["mico_elbow_roll_joint"]] / TO_RAD;
        velocity.Actuator5 = point.velocities[joints["mico_wrist_roll_joint"]] / TO_RAD;
        velocity.Actuator6 = point.velocities[joints["mico_hand_roll_joint"]] / TO_RAD;   */


        AngularInfo velocity;
        velocity.Actuator1 = point.velocities[joints["mico_shoulder_yaw_joint"]];
        velocity.Actuator2 = point.velocities[joints["mico_shoulder_pitch_joint"]];
        velocity.Actuator3 = point.velocities[joints["mico_elbow_pitch_joint"]];
        velocity.Actuator4 = point.velocities[joints["mico_elbow_roll_joint"]];
        velocity.Actuator5 = point.velocities[joints["mico_wrist_roll_joint"]];
        velocity.Actuator6 = point.velocities[joints["mico_hand_roll_joint"]];

        
        curTarget.Actuator1 += actuator1Correction; 
        curTarget.Actuator2 += actuator2Correction;
        curTarget.Actuator3 += actuator3Correction;
        curTarget.Actuator4 += actuator4Correction;
        curTarget.Actuator5 += actuator5Correction;
        curTarget.Actuator6 += actuator6Correction;

        // Start executing the goal
    //    arm.SetVelocities(velocity);
        arm.setTrajectory(curTarget);

        // Output some debugging info
        ROS_INFO_STREAM("Target Angles:");
        ROS_INFO_STREAM("Actuator 1: " << curTarget.Actuator1);
        ROS_INFO_STREAM("Actuator 2: " << curTarget.Actuator2);
        ROS_INFO_STREAM("Actuator 3: " << curTarget.Actuator3);
        ROS_INFO_STREAM("Actuator 4: " << curTarget.Actuator4);
        ROS_INFO_STREAM("Actuator 5: " << curTarget.Actuator5);
        ROS_INFO_STREAM("Actuator 6: " << curTarget.Actuator6 <<" \n");

/*
        ROS_INFO_STREAM("Velocities:");
        ROS_INFO_STREAM("Actuator 1: " << velocity.Actuator1);
        ROS_INFO_STREAM("Actuator 2: " << velocity.Actuator2);
        ROS_INFO_STREAM("Actuator 3: " << velocity.Actuator3);
        ROS_INFO_STREAM("Actuator 4: " << velocity.Actuator4);
        ROS_INFO_STREAM("Actuator 5: " << velocity.Actuator5);
        ROS_INFO_STREAM("Actuator 6: " << velocity.Actuator6 << "\n");
*/
    }

  //  arm.SetTrajectory(vAngles);

        // Output some debugging info
  /*      ROS_INFO_STREAM("Actuator 1: " << curTarget.Actuator1 <<" \n");
        ROS_INFO_STREAM("Actuator 2: " << curTarget.Actuator2 <<" \n");
        ROS_INFO_STREAM("Actuator 3: " << curTarget.Actuator3 <<" \n");
        ROS_INFO_STREAM("Actuator 4: " << curTarget.Actuator4 <<" \n");
        ROS_INFO_STREAM("Actuator 5: " << curTarget.Actuator5 <<" \n");
        ROS_INFO_STREAM("Actuator 6: " << curTarget.Actuator6 <<" \n");
*/
  /*      ROS_DEBUG_STREAM("Actuator Vel 1: " << point.velocities[joints["mico_shoulder_yaw_joint"]] <<" \n");
        ROS_DEBUG_STREAM("Actuator Vel 2: " << point.velocities[joints["mico_shoulder_pitch_joint"]] <<" \n");
        ROS_DEBUG_STREAM("Actuator Vel 3: " << point.velocities[joints["mico_elbow_pitch_joint"]] <<" \n");
        ROS_DEBUG_STREAM("Actuator Vel 4: " << point.velocities[joints["mico_elbow_roll_joint"]] <<" \n");
        ROS_DEBUG_STREAM("Actuator Vel 5: " << point.velocities[joints["mico_wrist_roll_joint"]] <<" \n");
        ROS_DEBUG_STREAM("Actuator Vel 6: " << point.velocities[joints["mico_hand_roll_joint"]] <<" \n");
*/
    while (true)
    {
        ros::spinOnce();
   
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_DEBUG_STREAM("Ros not Ok, Preemept Requested");
            arm.stopAPI();
            arm.startAPI();
            as_.setPreempted();
            return;
        }
    

        // Get current position
        arm.getJointAngles(curPosition);

        // Update current position for feedback and convert from degrees (Mico) to radians (ROS)
        feedback.actual.positions[joints["mico_shoulder_yaw_joint"]] = (180 - curPosition.Actuator1) * TO_RAD;
        feedback.actual.positions[joints["mico_shoulder_pitch_joint"]] = (curPosition.Actuator2 - 270) * TO_RAD;
        feedback.actual.positions[joints["mico_elbow_pitch_joint"]] = (90 - curPosition.Actuator3) * TO_RAD;
        feedback.actual.positions[joints["mico_elbow_roll_joint"]] = (180 - curPosition.Actuator4) * TO_RAD;
        feedback.actual.positions[joints["mico_wrist_roll_joint"]] = (180 - curPosition.Actuator5) * TO_RAD;
        feedback.actual.positions[joints["mico_hand_roll_joint"]] = (260 - curPosition.Actuator6) * TO_RAD;      

        // Calculate error (in radians)
        bool reachedJoints[6];
        bool reached = false;
        float error; 
        
        for (size_t j = 0; j < 6; ++j)
        {
            reachedJoints[j] = false;
            error = feedback.actual.positions[j] - feedback.desired.positions[j];
            feedback.error.positions[j] = error;
            //ROS_DEBUG_STREAM("error for joint " << j << " is" << feedback.error.positions[j]);
            
         /*   float tolerance = (p == trajectory.points.size() - 1) ?
                    //TODO: Fix Goal tolerance Issue
                    //goal->goal_tolerance[j].position
                    0.035
                :
                    //goal->path_tolerance[j].position
                    0.08
            ;
        */
        //    std::cout << "Error: " << error << "\n";
            float toleranceEnd = 0.03; // tolerance of the end point, needs different tolerance method.
            std::cout << "Error :" << j << ": " << error << "\n";
            
            if (error < 0.0f)
                error *= -1;
            
            if (error <= toleranceEnd)
            {
                std::cout << j << std::endl;
                reachedJoints[j] = true;
            }
        }
        
        for (size_t j = 0; j < 6; ++j)
        {
            if (reachedJoints[j] == false)
            {   
                std::cout << "breaking\n";
                break;
            }
            else if (j == 5 && reachedJoints[j] == true) 
                reached = true;
        }

        // If the arm stopped, the operation failed
        if (arm.isStopped())
        {
            ROS_DEBUG_STREAM("Operation Failed");
            as_.setAborted(result);
            return;
        }

        // Publish the feedback
        as_.publishFeedback(feedback);

        if (reached)
        {
            ROS_INFO("Goal reached!");
            break;
        }

        r.sleep();
    }
    // Done, proceeding to the next trajectory point
    
    ROS_INFO("Sending succes code!");
    // Goal reached
    result.error_code = 0; // SUCCESFUL = 0
    as_.setSucceeded(result);
}

} // Namespace jaco
