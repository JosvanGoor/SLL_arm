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
rospy.init_node('mico_arm_move', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

#Close gripper
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


####Move arm
group = moveit_commander.MoveGroupCommander("arm")
group.set_goal_tolerance(0.2)
display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)
print "============ Waiting for RVIZ..."
#rospy.sleep(10)
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

#new start pose
startPosDown = geometry_msgs.msg.Pose()
startPosDown.orientation.w = 1
startPosDown.orientation.x = 0.0
startPosDown.orientation.y = -0.000503295
startPosDown.orientation.z = -0.000386933
startPosDown.position.x = 0.18
startPosDown.position.y = 0.0233419
startPosDown.position.z = 0.25
group.set_pose_target(startPosDown)

plan1 = group.plan()
group.execute(plan1)
print "Pose: ",group.get_current_pose().pose

rospy.sleep(1)

#Object dimensions
#distance, width, depth, x,y (z?) or assume 
width = 0.08
depth = 0.08
#location 
curPose = group.get_current_pose().pose
x = 0.25
y = 0.02
#lotation edges object relative to current pose
goLeft = curPose.position.y - (y + (width/2))
goRight = curPose.position.y - (y - (width/2))
if(goRight<0):
  goRight = goRight *-1
if(goLeft<0):
  goLeft = goLeft *-1
goForward = (x + depth) - curPose.position.x

print "left ", goLeft, " right ", goRight, " forward ", goForward

minFrac = 0.9
maxCount = 3000
horizontalPadding = 0.075
verticalPadding = 0.08

fraction = 0
counter = 0
while(fraction<minFrac and counter<maxCount):
  rand = (random.random() - 0.5)/100
  counter +=1 
  group.clear_pose_targets()
  state = robot.get_current_state()
  group.set_start_state(state)
  waypoints = []
  wpose = group.get_current_pose().pose
  wpose.position.y += horizontalPadding + goLeft + rand
  waypoints.append(copy.deepcopy(wpose))
  #print "============ Waiting while RVIZ displays plan1..."
  (plan3, fraction) = group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.02,        # eef_step
                                1000.0)         # jump_threshold

if counter==maxCount:
  print "No success move 1a"
  exit(0)
print "========= Fraction: " ,fraction, " at trial ", counter
group.execute(plan3)
rospy.sleep(1)
print "Pose: ",group.get_current_pose().pose

fraction = 0
counter = 0
while(fraction<0.8 and counter<maxCount):
  rand = (random.random() - 0.5)/100
  counter +=1 
  group.clear_pose_targets()
  state = robot.get_current_state()
  group.set_start_state(state)
  waypoints = []
  wpose = group.get_current_pose().pose
  wpose.position.x += verticalPadding + goForward + rand
  waypoints.append(copy.deepcopy(wpose))

  #print "============ Waiting while RVIZ displays plan1..."
  (plan3, fraction) = group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.02,        # eef_step
                                1000.0)         # jump_threshold

if counter==maxCount:
  print "No success move 1b"
  exit(0)
print "========= Fraction: " ,fraction, " at trial ", counter
group.execute(plan3)
rospy.sleep(1)
print "Pose: ",group.get_current_pose().pose

fraction = 0
counter = 0
while(fraction<minFrac and counter<maxCount):
  rand = (random.random() - 0.5)/100
  counter +=1 
  group.clear_pose_targets()
  state = robot.get_current_state()
  group.set_start_state(state)
  waypoints = []
  wpose = group.get_current_pose().pose

  wpose.position.y -= horizontalPadding*2 + goRight + goLeft + rand
  waypoints.append(copy.deepcopy(wpose))
  #print "============ Waiting while RVIZ displays plan2..."
  (plan4, fraction) = group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.02,        # eef_step
                                1000.0)         # jump_threshold

if counter==maxCount:
  print "No success move 2"
  exit(0)
print "========= Fraction: " ,fraction, " at trial ", counter
group.execute(plan4)
rospy.sleep(1)
print "Pose: ",group.get_current_pose().pose

fraction = 0
counter = 0
while(fraction<minFrac and counter<maxCount):
  rand = (random.random() - 0.5)/100
  counter +=1 
  group.clear_pose_targets()
  state = robot.get_current_state()
  group.set_start_state(state)
  waypoints = []
  wpose = group.get_current_pose().pose
  wpose.position.x -= verticalPadding + goForward + rand
  waypoints.append(copy.deepcopy(wpose))
  wpose.position.y += horizontalPadding + goRight + rand
  waypoints.append(copy.deepcopy(wpose))

  #print "============ Waiting while RVIZ displays plan3..."
  (plan5, fraction) = group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.02,        # eef_step
                                1000.0)         # jump_threshold
  #rospy.sleep(2)

if counter==maxCount:
  print "No success move 3"
  exit(0)
print "========= Fraction: " ,fraction, " at trial ", counter
group.execute(plan5)
rospy.sleep(1)
print "Pose: ",group.get_current_pose().pose

collision_object = moveit_msgs.msg.CollisionObject()
moveit_commander.roscpp_shutdown()
