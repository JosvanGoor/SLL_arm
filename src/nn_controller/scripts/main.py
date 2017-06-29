import time
import signal, os
import math
import numpy as np
import cacla
from fk import FK # Forward Kinematics

class CACLA_arm(object):
    
    def __init__(self, base_path):
    
        self.zeroSpeed = 0.3
        self.basePath = base_path

        self.cacla_args = {'sigma':(0.01, 36.0), 'epsilon':(0.01, 1.0), 'alpha':(0.0001,0.0001), 'beta':(0.00005, 0.00005) \
                ,'discount_factor':0.98, 'random_decay': 0.999, 'var_beta': 0.0, 'learning_decay':1.0 \
                , 'explore_strategy':1}
        
        # Network not including input
        self.network = [500, 6]
        self.CACLA = cacla.Cacla(self.basePath, self.network, **self.cacla_args)

        # Rewards
        self.fixedPositive = 1;
        self.fixedNegative = -1;
        self.goalFound = 200;
        self.failed = -200;
        
        self.currentPosition1 = 0.0
        self.currentPosition2 = 180.0
        self.currentPosition3 = 180.0
        self.currentPosition4 = 0.0
        self.currentPosition5 = 0.0
        self.currentPosition6 = 0.0
        
        self.fk = FK()
        
     
    def writeConfig(self, goalPositions):
        f = open(os.path.join(self.basePath, "config"), 'w')
        
        f.write("Network: " + str(self.network) + '\n')
        f.write("Rewards:" + str(self.fixedPositive) + ", " + str(self.fixedNegative) + ", " + str(self.goalFound) + ", " + str(self.failed) + '\n')
        f.write("Goals: " + str(goalPositions))
        f.close()   
        
    def reset(self):
        self.prevDistance = -999 # just a init value which cannot be zero
        
        self.currentPosition1 = 0.0
        self.currentPosition2 = 180.0
        self.currentPosition3 = 180.0
        self.currentPosition4 = 0.0
        self.currentPosition5 = 0.0
        self.currentPosition6 = 0.0
        
        
    def wait(self, waitTime):
        beginTime = time.time()
        while True:
           # self.sim.checkPositions()
            curTime = time.time()    
            if curTime - beginTime >= waitTime:
                break
    
    def getPosition(self):
        return self.currentPosition1, self.currentPosition2, self.currentPosition3, self.currentPosition4, self.currentPosition5, self.currentPosition6
    
    def getEEF(self):
        return self.fk.getEEFPosition(joint1 = self.currentPosition1, joint2 = self.currentPosition2, joint3 = self.currentPosition3, \
                                      joint4 = self.currentPosition4, joint5 = self.currentPosition5, joint6 = self.currentPosition6)
    
    
    def checkJointLimit(self, joint):
        joint = 360 + joint if joint < 0.0 else \
                                (joint - 360.0 if joint >= 360.0 else joint)
        
        return joint
    
    
    def performAction(self, action, dt):
        action1 = action[0]
        action2 = action[1]
        action3 = action[2]
        action4 = action[3]
        action5 = action[4]
        action6 = action[5]
        
        if action1 > - self.zeroSpeed and action1 < self.zeroSpeed:
            action1 = 0 # No speed when it's almost zero
        
        if action2 > - self.zeroSpeed and action2 < self.zeroSpeed:
            action2 = 0 # No speed when it's almost zero
        
        if action3 > - self.zeroSpeed and action3 < self.zeroSpeed:
            action3 = 0 # No speed when it's almost zero
        
        if action4 > - self.zeroSpeed and action4 < self.zeroSpeed:
            action4 = 0 # No speed when it's almost zero
            
        if action5 > - self.zeroSpeed and action5 < self.zeroSpeed:
            action5 = 0 # No speed when it's almost zero
        
        if action6 > - self.zeroSpeed and action6 < self.zeroSpeed:
            action6 = 0 # No speed when it's almost zero
        
        self.currentPosition1 = self.currentPosition1 + action1 * dt  
        self.currentPosition2 = self.currentPosition2 + action2 * dt
        self.currentPosition3 = self.currentPosition3 + action3 * dt
        self.currentPosition4 = self.currentPosition4 + action4 * dt
        self.currentPosition5 = self.currentPosition5 + action5 * dt
        self.currentPosition6 = self.currentPosition6 + action6 * dt

        
        self.currentPosition1 = self.checkJointLimit(self.currentPosition1)
            
        if self.currentPosition2 < 50.0:
            self.currentPosition2 = 50.0;
        if self.currentPosition2 >= 310:
            self.currentPosition2 = 310;
            
        if self.currentPosition3 < 35.0:
            self.currentPosition3 = 35.0
        if self.currentPosition3 >= 325.0:
            self.currentPosition3 = 325.0
            
        self.currentPosition4 = self.checkJointLimit(self.currentPosition4)
        self.currentPosition5 = self.checkJointLimit(self.currentPosition5)
        self.currentPosition6 = self.checkJointLimit(self.currentPosition6)

            
        
    def get_reward(self, state, action):
        
        (x, y, z) = self.getEEF()
        distance = math.sqrt((state[0] - x)**2 + (state[1] - y)**2 + (state[2] - z)**2)      
        
        if self.prevDistance == -999:
            self.prevDistance = distance
            return 0, False;        
           
        if distance < 0.02:  # distance in m
            distance = 0
            
        reward = 0
        
        if distance < self.prevDistance: # getting closer to the goal
            reward = self.fixedPositive;
        elif distance >= self.prevDistance: # moving away from the goal or not moving at all
            reward = self.fixedNegative;
            
        #if distance < 0.2: # close to the object so give bigger rewards?
        #    reward *= 10
            
        if self.prevDistance == 0 and distance == 0 and (action[0] > -self.zeroSpeed and action[0] < self.zeroSpeed) and \
                                                        (action[1] > -self.zeroSpeed and action[1] < self.zeroSpeed) and \
                                                        (action[2] > -self.zeroSpeed and action[2] < self.zeroSpeed) and \
                                                        (action[3] > -self.zeroSpeed and action[3] < self.zeroSpeed) and \
                                                        (action[4] > -self.zeroSpeed and action[4] < self.zeroSpeed) and \
                                                        (action[5] > -self.zeroSpeed and action[5] < self.zeroSpeed):
            return self.goalFound, True
        
        self.prevDistance = distance;
        return reward, False
    
    def createGoals(self):
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
        
    def run(self):
        nr_epochs = 50000
        nr_iterations = 10000
    
        goalPositions = []
   
        successCount = 0
        
        nrEpochsDone = 0
        
       # joint1g = 90.0
       # joint2g = 250.0
        #joint3g = 250.0
                    
        #x,y,z = self.fk.getEEFPosition(joint1 = joint1g, joint2 = joint2g, joint3 = joint3g)
        
        #goalPositions = [[x, y, z]]
        #goalPositions = self.createGoals()
        goalPositions = [[0.0, -0.4, 0.3]]
        
        self.writeConfig(goalPositions) # write some stuff to the config file 
        
        for epoch in range(0, nr_epochs):
            
            #np.random.shuffle(goalPositions)
            if epoch % 100 == 0:
                print epoch
                   
            for g in goalPositions:
                nrEpochsDone += 1
                action = []
                terminal = False
                self.reset(); # reset currentPosition to 180
                
                for it in range(0, nr_iterations):                
                    state = []
                    for p in g:
                        state.append(p)
                    
                    joint1, joint2, joint3, joint4, joint5, joint6 = self.getPosition()
                    
                    state.append(joint1 / 360.0)
                    state.append(joint2 / 360.0)
                    state.append(joint3 / 360.0)
                    state.append(joint4 / 360.0)
                    state.append(joint5 / 360.0)
                    state.append(joint6 / 360.0)
    
                    reward, terminal = self.get_reward(state, action)
                                 
                    action = self.CACLA.run(state, reward, terminal)                    
                    
                    self.performAction(action, 0.02)
                    
                    if terminal:
                        #print 'Total iterations:', it
                        successCount += 1             
                        break;
                    
                if nrEpochsDone % 100 == 0:
                    print 'Nr. of successes:', successCount, "/", nrEpochsDone, ' = ', float(successCount) / nrEpochsDone
                   
                if terminal == False:
                    state = []
                    for p in g:
                        state.append(p)
                    
                    joint1, joint2, joint3, joint4, joint5, joint6 = self.getPosition()
                    
                    state.append(joint1 / 360.0)
                    state.append(joint2 / 360.0)
                    state.append(joint3 / 360.0)
                    state.append(joint4 / 360.0)
                    state.append(joint5 / 360.0)
                    state.append(joint6 / 360.0)
    
                    reward = self.failed
                            
                    action = self.CACLA.run(state, reward, True)
                
                self.CACLA.save()            
    
if __name__ == '__main__':
    
    for i in range(1, 100):
        base_path = '/home/rik/sudo/ros/catkin_ws/src/nn_controller/scripts/nnController' + str(i)
    
        if not os.path.exists(base_path):
            os.makedirs(base_path)
            break
    
    cacla = CACLA_arm(base_path)
    cacla.run();