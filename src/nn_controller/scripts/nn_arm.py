from fann import NeuralNetwork
import time
import signal, os
import math
import rospy
from vrep_test.srv import *
from vrep_test.msg import SetJointVelocity
from std_msgs.msg import Float32
from fk import FK # Forward Kinematics
import numpy as np

def wait(waitTime):
    beginTime = time.time()
    while True:
        sim.checkPositions()
        curTime = time.time()    
        if curTime - beginTime >= waitTime:
            break;

def createGoals():
    stepSize = 0.05
    
    goalPositions = []
    x = np.arange(-0.2, 0.2 + 0.01, stepSize)
    y = np.arange(-0.3, -0.2 + 0.01, stepSize)
    z = np.arange(0.2, 0.5 + 0.01, stepSize)
    
    for ix in x:
        for iy in y:
            for iz in z:
                goalPositions.append([ix, iy, iz])   
    
    print 'Nr of goal positions:', len(goalPositions)
    return goalPositions           

print 'Waiting for service'
rospy.wait_for_service("getJointPosition");
#rospy.wait_for_service("setJointVelocity");
print 'Found service'

getJointPositions = rospy.ServiceProxy("getJointPosition", GetJointPosition);
getEEFPosition = rospy.ServiceProxy("getEEFPosition", GetEEFPosition)
#setJointVelocity = rospy.ServiceProxy("setJointVelocity", SetJointVelocity);

setJointVelocity = rospy.Publisher("setJointVelocity", SetJointVelocity, queue_size=1);
rospy.init_node('arm_tester', anonymous=True)
time.sleep(2);

nnet = NeuralNetwork()
nnet.load(path='/home/rik/sudo/ros/catkin_ws/src/nn_controller/scripts/nnController1/actor_float.net');

jointNr = [2]
currentPosition = 0.0
    
step_size = 0.01
zeroSpeed = 0.3
ep = 1

#joint1g = 20.0
#joint2g = 210.0
#joint3g = 200.0

#fk = FK()
#x,y,z = fk.getEEFPosition(joint1 = joint1g, joint2 = joint2g, joint3 = joint3g)
#goalPositions = [[x, y, z]]

goalPositions = [[0.0, -0.4, 0.3]]

g = goalPositions[0] 
zeroCount = 0;  

print 'Goal:', g

while True:
    
    jointPositions = getJointPositions()  
    currentPosition1 = jointPositions.joint1 / 360.0      
    currentPosition2 = jointPositions.joint2 / 360.0
    currentPosition3 = jointPositions.joint3 / 360.0
    currentPosition4 = jointPositions.joint4 / 360.0      
    currentPosition5 = jointPositions.joint5 / 360.0
    currentPosition6 = jointPositions.joint6 / 360.0
    
    state = []
    for p in g:
        state.append(p)
    state.append(currentPosition1)
    state.append(currentPosition2) 
    state.append(currentPosition3)
    state.append(currentPosition4)
    state.append(currentPosition5) 
    state.append(currentPosition6)
   # input = [goalPosition, currentPosition]
    output = nnet.run(state)
    
    action1 = output[0]
    if action1 > -zeroSpeed and action1 < zeroSpeed:
        action1 = 0
    
    action2 = output[1]
    if action2 > -zeroSpeed and action2 < zeroSpeed:
        action2 = 0
 
    action3 = output[2]
    if action3 > -zeroSpeed and action3 < zeroSpeed:
        action3 = 0   
    
    action4 = output[3]
    if action4 > -zeroSpeed and action4 < zeroSpeed:
        action4 = 0  
        
    action5 = output[4]
    if action5 > -zeroSpeed and action5 < zeroSpeed:
        action5 = 0 
    
    action6 = output[5]
    if action6 > -zeroSpeed and action6 < zeroSpeed:
        action6 = 0   
    
    eef = getEEFPosition()
    
    distance = math.sqrt((g[0] - eef.x)**2 + (g[1] - eef.y)**2 + (g[2] - eef.z)**2)  
    print distance  

 #   print action
    velocity = SetJointVelocity()
    velocity.joint1 = action1
    velocity.joint2 = action2
    velocity.joint3 = action3
    velocity.joint4 = action4
    velocity.joint5 = action5
    velocity.joint6 = action6
    
    setJointVelocity.publish(velocity)
    time.sleep(0.005);
   # wait(step_size) # step for when taking the new value
   # sim.stopJointVelocity()
    



