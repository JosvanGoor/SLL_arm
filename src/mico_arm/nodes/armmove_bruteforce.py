#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('mico_arm_python', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")

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

# roll = 90;
# pitch = 40;
# yaw = 0;
# quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw);

# wpose.orientation.x = quaternion[0];
# wpose.orientation.y = quaternion[1];
# wpose.orientation.z = quaternion[2];
# wpose.orientation.w = quaternion[3];

# wpose.position.x = 0.38
# wpose.position.y = 0.06
# wpose.position.z = 0.4
# group.set_pose_target(wpose)

  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group 
  ## to actually move the robot

# startPosDown = geometry_msgs.msg.Pose()
# startPosDown.orientation.w = 1
# startPosDown.orientation.x = 0.0
# startPosDown.orientation.y = 0.0
# startPosDown.orientation.z = 0.0
# startPosDown.position.x = 0.189
# startPosDown.position.y = 0.063
# startPosDown.position.z = 0.22
# group.set_pose_target(startPosDown)

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

minFrac = 0.95
maxCount = 2000

fraction = 0
counter = 0
while(fraction<minFrac and counter<maxCount):
  counter +=1 
  group.clear_pose_targets()
  state = robot.get_current_state()
  group.set_start_state(state)
  waypoints = []
  wpose = group.get_current_pose().pose
  wpose.position.y += 0.2
  waypoints.append(copy.deepcopy(wpose))
  wpose.position.x += 0.2
  waypoints.append(copy.deepcopy(wpose))

  #print "============ Waiting while RVIZ displays plan1..."
  (plan3, fraction) = group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.02,        # eef_step
                                10.0)         # jump_threshold
#rospy.sleep(2)

if counter==maxCount:
  print "No success move 1"
  exit(0)
print "========= Fraction: " ,fraction
group.execute(plan3)
rospy.sleep(1)
print "Pose: ",group.get_current_pose().pose

fraction = 0
counter = 0
while(fraction<minFrac and counter<maxCount):
  counter +=1 
  group.clear_pose_targets()
  state = robot.get_current_state()
  group.set_start_state(state)
  waypoints = []
  wpose = group.get_current_pose().pose

  wpose.position.y -= 0.5
  waypoints.append(copy.deepcopy(wpose))
  #print "============ Waiting while RVIZ displays plan2..."
  (plan4, fraction) = group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.02,        # eef_step
                                10.0)         # jump_threshold
#rospy.sleep(2)

if counter==maxCount:
  print "No success move 2"
  exit(0)
print "========= Fraction: " ,fraction
group.execute(plan4)
rospy.sleep(1)
print "Pose: ",group.get_current_pose().pose

fraction = 0
counter = 0
while(fraction<minFrac and counter<maxCount):
  counter +=1 
  group.clear_pose_targets()
  state = robot.get_current_state()
  group.set_start_state(state)
  waypoints = []
  wpose = group.get_current_pose().pose
  wpose.position.x -= 0.25
  waypoints.append(copy.deepcopy(wpose))
  wpose.position.y += 0.2
  waypoints.append(copy.deepcopy(wpose))

  #print "============ Waiting while RVIZ displays plan3..."
  (plan5, fraction) = group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.02,        # eef_step
                                10.0)         # jump_threshold
  #rospy.sleep(2)

if counter==maxCount:
  print "No success move 3"
  exit(0)
print "========= Fraction: " ,fraction
group.execute(plan5)
rospy.sleep(1)
print "Pose: ",group.get_current_pose().pose

collision_object = moveit_msgs.msg.CollisionObject()
moveit_commander.roscpp_shutdown()
