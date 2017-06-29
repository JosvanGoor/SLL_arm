import vrep
import time
import math
from multiprocessing import Process

class VREP():
    
    def __init__(self, scene = "testScene.ttt", path = "/home/rik/sudo/ros/catkin_ws/src/vrep_test/scenes/", ip="127.0.0.1", port=19997, waitUntilConnected=True, 
                doNotReconnectOnceDisconnect=False,connectionTimeOut=1000,commThreadCycleMs=5):
        self.clientID = vrep.simxStart(ip, port, waitUntilConnected, doNotReconnectOnceDisconnect, connectionTimeOut, commThreadCycleMs)
        
        if self.clientID == -1:
            print 'Could not connect with V-REP Simulation'
            exit(0)
        
        print 'Connection made!'
        
        self.jointName = "Mico_joint"
       # self.jointName = "LBR4p_joint"
        self.jointHandles = []
        self.stopSimulation()
        time.sleep(0.5)
        self.loadScene(path, scene)
        
    
    def __del__(self):
        self.stopSimulation()
        vrep.simxFinish(self.clientID)
        print 'Disconnected with V-REP Simulation'
    
    
    def loadScene(self, path="/home/rik/sudo/ros/catkin_ws/src/vrep_test/scenes/", scene="testScene.ttt"):
        returnCode = vrep.simxLoadScene(self.clientID, path+scene,0,vrep.simx_opmode_oneshot_wait)
        
        if returnCode != 0:
            print 'Could not load scene', returnCode
        else:
            print 'Scene loaded'
            
        # also get the handles for the joints
        for j in range(1,7):
            returnCode, handle = vrep.simxGetObjectHandle(self.clientID, self.jointName+str(j), vrep.simx_opmode_oneshot_wait)
            if returnCode == 0:
                self.jointHandles.append(handle)
            else:
                print 'Could not get jointHandle', returnCode
                
        # get the handle of the dummy position
        returnCode, self.gripperHandle = vrep.simxGetObjectHandle(self.clientID, "EEF", vrep.simx_opmode_oneshot_wait)
        if returnCode == 0:
            pass;
        else:
            print 'Could not get EEFhandle', returnCode
            
        returnCode, self.frameHandle = vrep.simxGetObjectHandle(self.clientID, "Frame", vrep.simx_opmode_oneshot_wait)
        if returnCode == 0:
            pass
        else:
            print 'Could not get frameHandle', returnCode
            
    def startSimulation(self):
        returnCode = vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_oneshot)
        
        if returnCode == -1:
            print 'Could not start Simulation', returnCode
    
    def stopSimulation(self):
        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_oneshot_wait)
        
        
    def getJointPositions(self, joints = [1,2,3,4,5,6], show=False):  
        
        jointPositions = []
        
        for j in range(0, len(joints)):
            returnCode, angle = vrep.simxGetJointPosition(self.clientID, self.jointHandles[joints[j] - 1], vrep.simx_opmode_oneshot)
            
            if returnCode != 0 and returnCode != 1:
                print 'Something went wrong getting joint positions:', returnCode
              
            # round the floating point to 4 decimals to reduce state space?  
            jointPositions.append(round(math.degrees(angle),4))
        
        return jointPositions   
        
            
    def setJointVelocity(self, joints = [1,2,3,4,5,6], velocity = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]):   
                
        for j in range(0, len(joints)):
            returnCode = vrep.simxSetJointTargetVelocity(self.clientID, self.jointHandles[joints[j] - 1], math.radians(velocity[j]), vrep.simx_opmode_oneshot)
            
            if returnCode != 0 and returnCode != 1:
                print 'Something went wrong sending joint velocity:', returnCode
                
    def stopJointVelocity(self):
        
        for j in range(0, len(self.jointHandles)):
            returnCode = vrep.simxSetJointTargetVelocity(self.clientID, self.jointHandles[j], math.radians(0.0), vrep.simx_opmode_oneshot_wait)
            
            if returnCode != 0 and returnCode != 1:
                print 'Something went wrong stopping:', returnCode
    
    
    def setJointPosition(self, joints = [1,2,3,4,5,6], position = [-180, 180, 180, 180, -180,-180]):    
        
        #vrep.simxPauseSimulation(self.clientID, vrep.simx_opmode_oneshot_wait)
        #self.setJointVelocity([2,3], [0.01,0.01]) # these joints are not cyclic, and therefore somehow transform  to a weird position.
        for j in range(0, len(joints)):
            returnCode = vrep.simxSetJointPosition(self.clientID, self.jointHandles[joints[j] - 1], math.radians(position[j]), vrep.simx_opmode_oneshot)
        
            if returnCode != 0 and returnCode != 1:
                print 'Something went wrong sending joint positions:', returnCode

    
    
    def getEEFPosition(self):
        returnCode, position = vrep.simxGetObjectPosition(self.clientID, self.gripperHandle, self.frameHandle, vrep.simx_opmode_oneshot_wait)   
        return position
         
        
        
        
         
    