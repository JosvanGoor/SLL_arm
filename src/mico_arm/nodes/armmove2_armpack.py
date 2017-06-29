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
group = moveit_commander.MoveGroupCommander("mico_arm")

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
#plan1 = group.plan()


rospy.sleep(5)

waypoints = []

# # start with the current pose
waypoints.append(group.get_current_pose().pose)

#first orient gripper and move forward (+x)
wpose = geometry_msgs.msg.Pose()
wpose.orientation.w = waypoints[0].orientation.w
wpose.orientation.x = waypoints[0].orientation.x
wpose.orientation.y = waypoints[0].orientation.y
wpose.orientation.z = waypoints[0].orientation.z

wpose.position.x = waypoints[0].position.x + 0.1
wpose.position.y = waypoints[0].position.y
wpose.position.z = waypoints[0].position.z
waypoints.append(copy.deepcopy(wpose))

wpose.position.z += 0.1
waypoints.append(copy.deepcopy(wpose))

wpose.orientation.w += 0.3
waypoints.append(copy.deepcopy(wpose))

wpose.position.z += 0.1
waypoints.append(copy.deepcopy(wpose))
print "============ Waiting while RVIZ displays plan1..."
(plan3, fraction) = group.compute_cartesian_path(
                              waypoints,   # waypoints to follow
                              0.01,        # eef_step
                              0.0)         # jump_threshold

print "============ Publishing plan"
#group.go(wait=False)

collision_object = moveit_msgs.msg.CollisionObject()
moveit_commander.roscpp_shutdown()
