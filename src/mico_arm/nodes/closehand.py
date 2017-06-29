#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import random

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('mico_close_hand', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("gripper")

display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)

group_variable_values = group.get_current_joint_values()

group_variable_values[0] = 0.6
group_variable_values[1] = 0.6
group.set_joint_value_target(group_variable_values)
plan2 = group.plan()
print "Closing gripper"
group.execute(plan2)

collision_object = moveit_msgs.msg.CollisionObject()
moveit_commander.roscpp_shutdown()
