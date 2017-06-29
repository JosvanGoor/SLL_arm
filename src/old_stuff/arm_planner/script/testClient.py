#! /usr/bin/env python

import roslib
roslib.load_manifest('arm_planner')
import rospy
import actionlib

from arm_planner.msg import goalAction, goalGoal
from jaco_msgs.msg import *

if __name__ == '__main__':
    rospy.init_node('arm_planner_client')
    client = actionlib.SimpleActionClient('arm_planning', goalAction)
    client.wait_for_server()

    goal = goalGoal()
    # Fill in the goal here
    goal.x = 0.2;
    goal.y = -0.4;
    goal.z = 0.40;
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(60.0))
    
    grasping = actionlib.SimpleActionClient("grasping", graspingAction)
    print 'waiting for grasping server'
    grasping.wait_for_server()
    
    goal = graspingGoal()
    goal.object.function = "putDown"
    goal.object.x = 0.1;
    goal.object.y = -0.45;
    goal.object.z = 0.20;
    
    
    grasping.send_goal(goal)
    grasping.wait_for_result(rospy.Duration.from_sec(60.0))
