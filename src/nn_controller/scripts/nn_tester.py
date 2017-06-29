from simulator import VREP
from fann import NeuralNetwork
import time
import signal, os
import math
import numpy as np
from fk import FK


def stopSimulation():
    vrep.stopSimulation()
    
def wait(waitTime):
    beginTime = time.time()
    while True:
        #sim.checkPositions()
        curTime = time.time()    
        if curTime - beginTime >= waitTime:
            break;
        
def checkJointLimit(joint):
        joint = 360 + joint if joint < 0.0 else \
                                (joint - 360.0 if joint >= 360.0 else joint)
        
        return joint

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
        
# Catch Ctr-c and Ctr-z messages to stop simulation    
signal.signal(signal.SIGINT, stopSimulation)
signal.signal(signal.SIGTERM, stopSimulation)

port = 19997
sim = VREP(port=port)

nnet = NeuralNetwork()
nnet.load(path='/home/rik/sudo/ros/catkin_ws/src/nn_controller/scripts/nnController2/actor_float.net');

zeroSpeed = 0.3
ep = 1

fk = FK()

    
print 'Starting LEARNED run'

goalPositions = [[0.0, -0.4, 0.3]]

avgDistance = 0
nrSuccess = 0
nrFailed = 0

prevDistance = 0

for g in goalPositions:  
        sim.startSimulation()           
        state = []   
            
        doneCount = 0
        
        for it in range(0,5000):
            state = []
            for p in g:
                state.append(p)
            
            joints = sim.getJointPositions([1,2,3,4,5,6])

            joints[0] = checkJointLimit(joints[0]);
            joints[3] = checkJointLimit(joints[3]);
            joints[4] = checkJointLimit(joints[4]);
            joints[5] = checkJointLimit(joints[5]);
            
            #print joints[0]
            state.append(joints[0] / 360.0)
            state.append(joints[1] / 360.0)  
            state.append(joints[2] / 360.0)
            state.append(joints[3] / 360.0)
            state.append(joints[4] / 360.0)  
            state.append(joints[5] / 360.0)
    
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
               
            sim.setJointVelocity([1,2,3,4,5,6], [action1, action2, action3, action4, action5, action6])
            
            
            position =  sim.getEEFPosition()
            distance = math.sqrt((g[0] - position[0])**2 + (g[1] - position[1])**2 + (g[2] - position[2])**2)
            
            print distance
            
            if distance <= 0.02 and (math.fabs(distance - prevDistance) < 0.0001):
                doneCount += 1
            else:
                doneCount = 0
                
            prevDistance = distance;
            
            
            #print distance
            if doneCount == 100:
                avgDistance += distance
                nrSuccess += 1
                print nrSuccess
                print '-----------------DONE--------------'
                #time.sleep(2)
                break
            
            #print 'Time:', time.time() - prevTime
            #prevTime = time.time()
            
          #  wait(step_size) # step for when taking the new value
          #  sim.stopJointVelocity()
    
        sim.stopSimulation() 
        time.sleep(0.5)

print 'Avg distance:', avgDistance / nrSuccess
print 'Nr Success:', nrSuccess
print 'Nr Failed:', len(goalPositions) - nrSuccess