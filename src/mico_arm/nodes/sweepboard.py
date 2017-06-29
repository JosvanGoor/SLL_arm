#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
import tf
import random

from colorblob.msg import Bbox
from colorblob.msg import BboxArray

def startPosition():
  global group
  # Close gripper
  # group = moveit_commander.MoveGroupCommander("gripper")

  # display_trajectory_publisher = rospy.Publisher(
  #                                     '/move_group/display_planned_path',
  #                                     moveit_msgs.msg.DisplayTrajectory)

  # group_variable_values = group.get_current_joint_values()

  # group_variable_values[0] = 0.6
  # group_variable_values[1] = 0.6
  # group.set_joint_value_target(group_variable_values)
  # plan2 = group.plan()
  # print "============ Closing gripper"
  # group.execute(plan2)

  ###Move arm
  group = moveit_commander.MoveGroupCommander("arm")
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)

  print "============ Moving to start position"
  wpose = geometry_msgs.msg.Pose()

  #new start pose
  startPosDown = geometry_msgs.msg.Pose()
  startPosDown.orientation.w = -0.495465
  startPosDown.orientation.x = 0.504313
  startPosDown.orientation.y = 0.50426
  startPosDown.orientation.z = -0.495889
  startPosDown.position.x = 0.35
  startPosDown.position.y = 0.0
  startPosDown.position.z = 0.236591
  group.set_pose_target(startPosDown)

  plan1 = group.plan()
  group.execute(plan1)
  print "Pose: ",group.get_current_pose().pose

def sweep(w,h):
  global group
  print "Moving forward"

  for i in range(11):
    startPosDown = group.get_current_pose().pose
    startPosDown.position.x = 0.45 - (i/100)
    group.set_pose_target(startPosDown)

    plan1 = group.plan()
    worked = group.execute(plan1)
    if worked:
      break
  print "Pose: ",group.get_current_pose().pose
  print i
  rospy.sleep(2)
  # w = w*1.2
  # h = h*1.2
  #group.set_goal_tolerance(0.05)
  minFrac = 0.2
  fraction = 0
  counter = 0
  
  print "Sweeping"
  group.clear_pose_targets()
  group_variable_values = group.get_current_joint_values()

  group_variable_values[-1] -= 0.1
  group.set_joint_value_target(group_variable_values)
  plan2 = group.plan()
  group.execute(plan2)

  group.clear_pose_targets()
  group_variable_values = group.get_current_joint_values()

  group_variable_values[-1] += 0.1
  group.set_joint_value_target(group_variable_values)
  plan2 = group.plan()
  group.execute(plan2)

  # startPosDown = group.get_current_pose().pose
  # startPosDown.position.y -= 0.02
  # group.set_pose_target(startPosDown)
  # plan = group.plan()
  # group.execute(plan)

  # group.clear_pose_targets()
  # startPosDown = group.get_current_pose().pose
  # startPosDown.position.y += 0.02
  # group.set_pose_target(startPosDown)
  # plan = group.plan()
  # group.execute(plan)

  print "Moving back"
  startPosDown = group.get_current_pose().pose
  startPosDown.position.x = 0.35
  group.set_pose_target(startPosDown)

  plan1 = group.plan()
  group.execute(plan1)
  print "Pose: ",group.get_current_pose().pose
  

  rospy.sleep(2)


def moveTo(coordinate): #x,y,w,h
  global group
  group.set_goal_tolerance(0.05)
  minFrac = 0.9
  posY = coordinate[0]
  posZ = coordinate[1]
  group.clear_pose_targets()
  wpose = group.get_current_pose().pose
  wpose.position.y = posY
  wpose.position.z = posZ
  group.set_pose_target(wpose)
  plan = group.plan()
  group.execute(plan)
  rospy.sleep(2)
  sweep(coordinate[2], coordinate[3])


  # fraction = 0
  # counter = 0
  # while(fraction<minFrac and counter<10):
  #   rand = (random.random() - 0.5)/100
  #   counter +=1 
  #   group.clear_pose_targets()
  #   state = robot.get_current_state()
  #   group.set_start_state(state)
  #   waypoints = []
  #   wpose = group.get_current_pose().pose
  #   wpose.position.y = posY + rand
  #   wpose.position.z = posZ + rand
  #   waypoints.append(copy.deepcopy(wpose))
  #   #print "============ Waiting while RVIZ displays plan1..."
  #   (plan3, fraction) = group.compute_cartesian_path(
  #                                 waypoints,   # waypoints to follow
  #                                 0.02,        # eef_step
  #                                 1000.0)         # jump_threshold
  # if counter!=10:
  #   print "Move successful"
  #   group.execute(plan3)
  #   rospy.sleep(2)
  #   sweep(coordinate[2], coordinate[3])
  #   return True;
  # print "Moving back and trying again"
  # fraction = 0
  # counter = 0
  # while(fraction<minFrac and counter<2000):
  #   rand = (random.random() - 0.5)/100
  #   counter +=1 
  #   group.clear_pose_targets()
  #   state = robot.get_current_state()
  #   group.set_start_state(state)
  #   waypoints = []
  #   wpose = group.get_current_pose().pose
  #   wpose.position.x -= 0.1
  #   waypoints.append(copy.deepcopy(wpose))
  #   wpose.position.y = posY + rand
  #   wpose.position.z = posZ + rand
  #   waypoints.append(copy.deepcopy(wpose))
  #   wpose.position.x += 0.1
  #   waypoints.append(copy.deepcopy(wpose))
  #   #print "============ Waiting while RVIZ displays plan1..."
  #   (plan3, fraction) = group.compute_cartesian_path(
  #                                 waypoints,   # waypoints to follow
  #                                 0.02,        # eef_step
  #                                 1000.0)         # jump_threshold
  # if counter==2000:
  #   print "Move unsuccessful, continuing to next"
  #   return False
  # else:
  #   group.execute(plan3)
  #   rospy.sleep(2)
  #   sweep(coordinate[2], coordinate[3])
  return True

def placeObstacle(coordinates): #places obstacles in place of every blue location
  pub_co = rospy.Publisher('/collision_object', moveit_msgs.msg.CollisionObject, queue_size=100)
  collisionObjects = []
  #gets x,y,w,h
  global scene
  global robot

  dimensions = [0,0,0]
  dimensions[0] = 0.05
  dimensions[1] = coordinates[2]
  dimensions[2] = coordinates[3]

  collision_object = moveit_msgs.msg.CollisionObject()
  collision_object.id = "blue_blob"
  collision_object.header.frame_id = group.get_planning_frame()
  box = shape_msgs.msg.SolidPrimitive()
  box.type = shape_msgs.msg.SolidPrimitive().BOX
  box.dimensions = list(dimensions)

  collision_object.primitives = [box]

  boxPose = geometry_msgs.msg.Pose()
  boxPose.orientation.w = 1.0
  boxPose.position.x = 0.55 #set distance (between arm base and wall)
  boxPose.position.y = coordinates[0]
  boxPose.position.z = coordinates[1]

  collision_object.primitive_poses = [boxPose]

  pub_co.publish(collision_object)


def getMessage():
  msg = rospy.wait_for_message("colorblob", BboxArray)
  print msg.size
  blobs = []
  for box in msg.bboxes:
    blobs.append([box.tag,box.x,box.y, box.width, box.height])
  print "Blobs: ", blobs
  return blobs


def main():
  global group
  group = moveit_commander.MoveGroupCommander("gripper")
  #place obstacle behind it to avoid hitting the camera
  pub_co = rospy.Publisher('/collision_object', moveit_msgs.msg.CollisionObject, queue_size=100)
  collisionObjects = []
  #gets x,y,w,h
  global scene
  global robot

  dimensions = [0.1,0,0]
  dimensions[0] = 0.05
  dimensions[1] = 0.2
  dimensions[2] = 0.6

  collision_object = moveit_msgs.msg.CollisionObject()
  collision_object.id = "camera_stand"
  collision_object.header.frame_id = group.get_planning_frame()
  box = shape_msgs.msg.SolidPrimitive()
  box.type = shape_msgs.msg.SolidPrimitive().BOX
  box.dimensions = list(dimensions)

  collision_object.primitives = [box]

  boxPose = geometry_msgs.msg.Pose()
  boxPose.orientation.w = 1.0
  boxPose.position.x = -0.35 #set distance (between arm base and wall)
  boxPose.position.y = 0.0
  boxPose.position.z = 0.0

  collision_object.primitive_poses = [boxPose]

  pub_co.publish(collision_object)

  #320 pixels is approximately 1 meter (very roughly measured)
  #resolution is 640x480
  scale = 1.29/640
  targets = []
  obstacles = []
  print "Getting messages "
  for blob in getMessage():
    #scale still very wrong
    x = (320-(blob[1]+(blob[3]/2)))*scale
    y = (480-(blob[2]+(blob[4]/2)))*scale + 0.17
    width = blob[3]*scale
    height = blob[4]*scale
    if blob[0] == 'colorblob_red':
      targets.append([x,y,width,height])
    elif blob[0] == 'colorblob_blue':
      obstacles.append([x,y,width,height])

  startPosition()

  print "=== Processed all objects"
  print "Obstacles: ", obstacles
  for o in obstacles:
    placeObstacle(o)

  print "Placed obstacles"
  successRate = 0
  for t in targets:
    print t
    if moveTo(t):
      successRate+=1
  print "Successfully moved to ", successRate, " out of ", len(targets), " targets"

  collision_object = moveit_msgs.msg.CollisionObject()
  moveit_commander.roscpp_shutdown()


if __name__=='__main__':
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('mico_arm_move', anonymous=True)
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  try:
    main()
  except rospy.ROSInterruptException:
    pass