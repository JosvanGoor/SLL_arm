#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('mico_arm_python', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("mico_arm")

display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)
print "============ Waiting for RVIZ..."
rospy.sleep(10)
print "============ Starting tutorial "

print "============ Reference frame: %s" % group.get_planning_frame()
print "============ Reference frame: %s" % group.get_end_effector_link()

print "============ Robot Groups:"
print robot.get_group_names()

print "============ Printing robot state"
print robot.get_current_state()
print "============"


print "============ Generating plan 1"
wpose = geometry_msgs.msg.Pose()
wpose.orientation.w = -0.29
wpose.orientation.x = 0.64
wpose.orientation.y = 0.64
wpose.orientation.z = -0.29
wpose.position.x = 0.38
wpose.position.y = 0.06
wpose.position.z = 0.22
group.set_pose_target(wpose)

  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group 
  ## to actually move the robot
plan1 = group.plan()

print "============ Waiting while RVIZ displays plan1..."
rospy.sleep(5)

# waypoints = []

# # start with the current pose
# waypoints.append(group.get_current_pose().pose)

# # first orient gripper and move forward (+x)
# # wpose = geometry_msgs.msg.Pose()
# # wpose.orientation.w = 0.0
# # wpose.position.x = waypoints[0].position.x + 0.5
# # wpose.position.y = waypoints[0].position.y
# # wpose.position.z = waypoints[0].position.z
# # waypoints.append(copy.deepcopy(wpose))

# wpose = geometry_msgs.msg.Pose()
# wpose.orientation.w = -0.29
# wpose.orientation.x = 0.64
# wpose.orientation.y = 0.64
# wpose.orientation.z = -0.29
# wpose.position.x = 0.38
# wpose.position.y = 0.06
# wpose.position.z = 0.22
# waypoints.append(copy.deepcopy(wpose))

# # # second move down
# # wpose.position.z += 0.05
# # waypoints.append(copy.deepcopy(wpose))

# # # third move to the side
# # wpose.position.y += 0.05
# # waypoints.append(copy.deepcopy(wpose))

# (plan3, fraction) = group.compute_cartesian_path(
#                              waypoints,   # waypoints to follow
#                              0.01,        # eef_step
#                              0.0)         # jump_threshold


print "============ Waiting while RVIZ displays plan3..."
rospy.sleep(5)

collision_object = moveit_msgs.msg.CollisionObject()
moveit_commander.roscpp_shutdown()