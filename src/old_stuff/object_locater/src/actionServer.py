import roslib; roslib.load_manifest('object_locater'); roslib.load_manifest('jaco_msgs')
import rospy
import actionlib
import sys
import cjson
import time
import signal
import math

from object_locater.msg import *
from jaco_msgs.msg import *

roslib.load_manifest('borg_pioneer')

from borg_pioneer.srv import *

interval = 1 #1 minutes


class ActionClient:
    def __init__(self):

        print "Wait for write memory"
        self.service_write = rospy.ServiceProxy('memory', MemorySrv)
        rospy.wait_for_service('memory')
        print "Write memory found"

        self.service_read = rospy.ServiceProxy('memory_read', MemoryReadSrv)
        rospy.wait_for_service('memory_read')

        self.client = actionlib.SimpleActionClient('locate_object', locateObjectAction)
        print 'Waiting for locate object server'
        self.client.wait_for_server()

        self.grasping = actionlib.SimpleActionClient("grasping", graspingAction)
        print 'waiting for grasping server'
        self.grasping.wait_for_server()

        self.result = None 

        self.state = "waitingForObject"
        self.previous_state = ""
        
        self.avoid_looping = False #To avoid a loop from Home position to Nav and back to Home
        
        self.external_detection = False #If external object recognition is working, skip detection here
        self.object = None #Object comming from outside

    def reinit(self):
        self.service_write = rospy.ServiceProxy('memory', MemorySrv)
        rospy.wait_for_service('memory')

        self.service_read = rospy.ServiceProxy('memory_read', MemoryReadSrv)
        rospy.wait_for_service('memory_read')
        print "memory found"
        self.state = "waitingForObject" # reset the state machine
        
    def update(self, signum, frame):

        if (self.state == "waitingForObject"): # read from memory
            try:
                memresult = self.service_read('get_last_observation',rospy.Time.now(),'readyToGrasp','')
                memresult = cjson.decode(memresult.json)
                
                if memresult:                       
                    print 'Mem results: ', memresult
                    if (memresult['start'] == True):
                        self.state = "moveingArmToHome"
                        try:
                            self.object = memresult['object']
                            self.external_detection = True
                            print "received object"
                        except Exception as e:
                            print e
                            self.external_detection = False 
                
            except Exception as e:
                print 'expection here:', e
                self.reinit()
                
        elif (self.state == "moveingArmToHome"):
            if not self.avoid_looping:
                self.service_write(rospy.Time.now(), "readyToGrasp", cjson.encode({'start': False}))
                print 'Moving arm to home position'
                goal = graspingGoal()
                goal.object.function = "fromNavToHome"
                
                self.grasping.send_goal(goal)
                
                self.grasping.wait_for_result(rospy.Duration.from_sec(40.0)) # time-out checking, if takes to long stop waiting... shouldn't take longer then 1 minute?
    
                if (self.grasping.get_state() == actionlib.GoalStatus.SUCCEEDED):
                        print 'In home position, ready to start'
                        if not self.external_detection:
                            self.state = "detectingObject"
                        else:
                            self.state = "grasping"
                        time.sleep(7) # moving without error check, very dangerous, but well.. we like to live like we program...
            else:
                self.service_write(rospy.Time.now(), "readyToGrasp", cjson.encode({'start': False}))
                if not self.external_detection:
                    self.state = "detectingObject"
                else:
                    self.state = "grasping"
            
        elif (self.state == "detectingObject"):
            print 'Detecting objects'        
            goal = locateObjectGoal()
            goal.startLocating = True
            self.client.send_goal(goal)
            self.client.wait_for_result(rospy.Duration.from_sec(40.0))  # use this for time-out checking, if takes to long stop waiting
            
            result = None        
            grasp = False
            
            if (self.client.get_state() == actionlib.GoalStatus.ABORTED):  # Could not find object after trying 10 times
                print 'aborted!' 
                try:
                    self.service_write(rospy.Time.now(), "objectDetection", cjson.encode("False"))
                except:
                    self.reinit()
                
                self.state = "waitingForObject"

                result = self.client.get_result()
                print 'Result: ', result.object.correct

            elif (self.client.get_state() == actionlib.GoalStatus.SUCCEEDED):  # has found one cluster and is ready for grasping
                print 'Succes!'
                self.result = self.client.get_result()
                self.state = "grasping"

                try:
                    self.service_write(rospy.Time.now(), "objectDetection", cjson.encode("True"))
                except:
                    self.reinit()

            elif (self.client.get_state() != actionlib.GoalStatus.SUCCEEDED): # most likely the time-out accord
                print 'Time out' 
            	# should not get result! just for testing stuff
                result = self.client.get_result()
                self.state = "waitingForObject" 

                try:
                    self.service_write(rospy.Time.now(), "objectDetection", cjson.encode("False"))
                except:
                    self.reinit() 

        elif (self.state == "grasping"):
            goal = graspingGoal()
            if not self.external_detection:
                goal = graspingGoal()
                goal.object.x = self.result.object.x
                goal.object.y = self.result.object.y
                goal.object.z = self.result.object.z
                goal.object.thetaX = self.result.object.thetaX
                goal.object.thetaY = self.result.object.thetaY
                goal.object.thetaZ = self.result.object.thetaZ
                goal.object.height = self.result.object.height
                goal.object.function = "grasp"
            else:
                print self.object
                goal.object.x = self.object['x']
                goal.object.y = self.object['y']
                goal.object.z = self.object['z'] - 0.02
                goal.object.thetaX = math.radians(90)
                goal.object.thetaY = 0
                goal.object.thetaZ = 0
                goal.object.height = self.object['z'] - 0.06
                
                goal.object.function = "grasp" 
                self.external_detection = False
                self.object = None
            print 'Going to grasp object'
           # goal.object.grasp = True;
            self.grasping.send_goal(goal)

            self.grasping.wait_for_result(rospy.Duration.from_sec(80.0)) # time-out checking, if takes to long stop waiting... shouldn't take longer then 1 minute?

            if (self.grasping.get_state() == actionlib.GoalStatus.ABORTED):
                result = self.grasping.get_result()
                print 'Aborted, something went wrong, moving to home position'

                goal.object.function = 'home'; # send it to home position

                self.grasping.send_goal(goal)
                self.grasping.wait_for_result(rospy.Duration.from_sec(80.0))
                
                self.avoid_looping = True

                if (self.grasping.get_state() == actionlib.GoalStatus.ABORTED):
                    print 'something went horribly wrong, arm is stuck and cannot move to home position'
                    try:
                        self.service_write(rospy.Time.now(), "grasping", cjson.encode("False"))
                    except:
               	 	    self.reinit()

                    self.state = "waitingForObject" # reset state machine

                elif (self.grasping.get_state() == actionlib.GoalStatus.SUCCEEDED):
                    print 'oke, back to home position,  ready to retry at different location?'
                    try:
                		self.service_write(rospy.Time.now(), "grasping", cjson.encode("False"))
                    except:
               	 		self.reinit()

                    self.state = "waitingForObject" # reset state machine
                    

                elif (self.grasping.get_state() != actionlib.GoalStatus.SUCCEEDED):
                    print 'something went horribly wrong, arm is stuck and cannot move to home position, after 1 min of trying'
                    try:
                		self.service_write(rospy.Time.now(), "grasping", cjson.encode("False"))
                    except:
               	 		self.reinit()

                    self.state = "waitingForObject" # reset state machine

            elif (self.grasping.get_state() == actionlib.GoalStatus.SUCCEEDED):
                result = self.grasping.get_result()
                print 'Succes'
                
                self.avoid_looping = False
                self.state = "moveToNavPosition"        
                
            elif (self.grasping.get_state() != actionlib.GoalStatus.SUCCEEDED):
                print 'time-out'
                try:
        	        self.service_write(rospy.Time.now(), "grasping", cjson.encode("False"))
                except:
             	    self.reinit()
                     
                self.avoid_looping = True
                self.state = "waitingForObject" # reset state machine
        
        elif (self.state == "moveToNavPosition"):
            goal = graspingGoal()
            goal.object.function = "navigation"
            
            self.grasping.send_goal(goal)
            
            self.grasping.wait_for_result(rospy.Duration.from_sec(40.0)) # time-out checking, if takes to long stop waiting... shouldn't take longer then 1 minute?

            if (self.grasping.get_state() == actionlib.GoalStatus.SUCCEEDED):
                    time.sleep(10) # again, living dangerously
                    try:
                        self.service_write(rospy.Time.now(), "grasping", cjson.encode("True"))
                    except:
                        self.reinit()
                    print 'In navigation position, ready to drive'
                    #self.state = "waitingForObject"
                    self.state = "waitingForDropOff" 
        
           
        elif (self.state == "waitingForDropOff"): # this can be put down or pour in basket
         
            try:
                memresult = self.service_read('get_last_observation',rospy.Time.now(),'readToDropOff','') # for pouring
            except Exception as e:
                self.reinit()
            else:
                memresult = cjson.decode(memresult.json)
                if memresult:      
                    if (memresult['start'] == True):
                		self.state = "dropOff"
                        
            try:
                memresult = self.service_read('get_last_observation',rospy.Time.now(),'readToPutDown','') # for putting it down
            except Exception as e:
                self.reinit()
            else:
                memresult = cjson.decode(memresult.json)
                if memresult:      
                    if (memresult['start'] == True):
                        self.state = "putDown"
                        print 'putting down the object'
            
        elif (self.state == "dropOff"):
            print 'Moving arm to home position'
            goal = graspingGoal()
            goal.object.function = "fromNavToHome"
            
            self.grasping.send_goal(goal)
            
            self.grasping.wait_for_result(rospy.Duration.from_sec(40.0)) # time-out checking, if takes to long stop waiting... shouldn't take longer then 1 minute?

            if (self.grasping.get_state() == actionlib.GoalStatus.SUCCEEDED):
                time.sleep(7) # moving without error check, very dangerous, but well.. we like to live like we program...
                print 'In home position, ready to start'
            
                goal = locateObjectGoal()
                goal.startLocating = True
                self.client.send_goal(goal)
                self.client.wait_for_result(rospy.Duration.from_sec(20.0))  # use this for time-out checking, if takes to long stop waiting
                result = None
    
                if (self.client.get_state() == actionlib.GoalStatus.ABORTED):  # Could not find object after trying 10 times
                    print 'aborted!' 
                    result = self.client.get_result()
                    print 'Result: ', result.object.correct
                    try:
                        self.service_write(rospy.Time.now(), "dropOff", cjson.encode("False"))
                    except:
                        self.reinit()
                    self.state = "waitingForDropoff"
    
                elif (self.client.get_state() == actionlib.GoalStatus.SUCCEEDED):  # has found one cluster and is ready for grasping
                    print 'Succes!'
                    result = self.client.get_result()
    
                    goal = graspingGoal()
                    goal.object.x = result.object.x
                    goal.object.y = result.object.y
                    goal.object.z = result.object.z 
                    goal.object.thetaX = result.object.thetaX
                    goal.object.thetaY = result.object.thetaY
                    goal.object.thetaZ = result.object.thetaZ
                    goal.object.height = result.object.height
                    goal.object.tableHeight = result.object.tableHeight
                    goal.object.xLeft = result.object.xLeft
                    goal.object.xRight = result.object.xRight
                    goal.object.function = "drop"
    
                    self.grasping.send_goal(goal)
    
                    self.grasping.wait_for_result(rospy.Duration.from_sec(60.0)) # time-out checking, if takes to long stop waiting... shouldn't take longer then 1 minute?
    
                    if (self.grasping.get_state() == actionlib.GoalStatus.ABORTED):
                        result = self.grasping.get_result()
                        print 'Aborted, something went wrong, moving to home position'
                        goal.object.function = "home"; # send it to home position
    
                        self.grasping.send_goal(goal)
                        self.grasping.wait_for_result(rospy.Duration.from_sec(60.0))
                        
                        try:
                            self.service_write(rospy.Time.now(), "dropOff", cjson.encode("False"))
                        except:
                            self.reinit()
                            
                        self.state = 'moveToNavigationAgain'
                        
                    elif (self.grasping.get_state() == actionlib.GoalStatus.SUCCEEDED):
                        print 'done dropping'
                        
                        try:
                            self.service_write(rospy.Time.now(), "dropOff", cjson.encode("True"))
                        except:
                            self.reinit()
    
                        self.state = 'moveToNavigationAgain'

            elif (self.client.get_state() != actionlib.GoalStatus.SUCCEEDED): # most likely the time-out accord
                print 'Time out' 
                self.state = "waitingForDropoff"
                
        elif (self.state == "putDown"):
            print 'Moving arm to home position'
            goal = graspingGoal()
            goal.object.function = "fromNavToHome"
            
            self.grasping.send_goal(goal)
            
            self.grasping.wait_for_result(rospy.Duration.from_sec(40.0)) # time-out checking, if takes to long stop waiting... shouldn't take longer then 1 minute?

            if (self.grasping.get_state() == actionlib.GoalStatus.SUCCEEDED):
                time.sleep(7) # moving without error check, very dangerous, but well.. we like to live like we program...
                print 'In home position, ready to start'
            
                print 'in state putting down'
                goal = locateObjectGoal()
                goal.findEmptySpace = True  # finds open space for object to place down
                
                self.client.send_goal(goal)
                self.client.wait_for_result(rospy.Duration.from_sec(60.0))  # use this for time-out checking, if takes to long stop waiting

    
                if (self.client.get_state() == actionlib.GoalStatus.ABORTED):  # Could not find object after trying 10 times
                    print 'aborted!' 
                    result = self.client.get_result()
                    print 'Result: ', result.object.correct
                    try:
                        self.service_write(rospy.Time.now(), "putDown", cjson.encode("False"))
                    except:
                        self.reinit()
                    self.state = "waitingForDropoff"
    
                elif (self.client.get_state() == actionlib.GoalStatus.SUCCEEDED):  # has found one cluster and is ready for grasping
                    print 'Found drop place'
                    print 'Placing object'
                    
                    result = self.client.get_result()
    
                    goal = graspingGoal()
                    goal.object.x = result.object.x
                    goal.object.y = result.object.y
                    goal.object.z = result.object.z 
                    goal.object.thetaX = result.object.thetaX
                    goal.object.thetaY = result.object.thetaY
                    goal.object.thetaZ = result.object.thetaZ
                    goal.object.height = result.object.height
                #   goal.object.tableHeight = result.object.tableHeight
                #    goal.object.xLeft = result.object.xLeft
                #    goal.object.xRight = result.object.xRight
                    goal.object.function = "putDown"
    
                    self.grasping.send_goal(goal)
                    self.grasping.wait_for_result(rospy.Duration.from_sec(60.0))
                    
                    if (self.grasping.get_state() == actionlib.GoalStatus.ABORTED):
                        result = self.grasping.get_result()
                        print 'Aborted, something went wrong, moving to home position'
                        goal.object.function = "home"; # send it to home position
    
                        self.grasping.send_goal(goal)
                        self.grasping.wait_for_result(rospy.Duration.from_sec(60.0))
                        
                        try:
                            self.service_write(rospy.Time.now(), "putDown", cjson.encode("False"))
                        except:
                            self.reinit()
                            
                        self.state = "done"
    
                    elif (self.grasping.get_state() == actionlib.GoalStatus.SUCCEEDED):
                        print 'done dropping'
                        goal.object.function = "home"; # send it to home position
    
                        self.grasping.send_goal(goal)
                        self.grasping.wait_for_result(rospy.Duration.from_sec(60.0))
                        
                        if (self.grasping.get_state() == actionlib.GoalStatus.ABORTED):
                            print 'something went horribly wrong, arm is stuck and cannot move to home position'
                            self.state = "moveToNavigationAgain"
                            
                        elif (self.grasping.get_state() == actionlib.GoalStatus.SUCCEEDED):
                            print 'oke, back to home position'
                            
                            try:
                                self.service_write(rospy.Time.now(), "putDown", cjson.encode("True"))
                            except:
                                self.reinit() 
                                
                            self.state = "moveToNavigationAgain" 
                        
                        self.state = 'moveToNavigationAgain'
                                                 
                    elif (self.grasping.get_state() != actionlib.GoalStatus.SUCCEEDED):
                        print 'time out!!!'
                        self.state = 'moveToNavigationAgain' 
    
                    #    self.state = "waitingForObject" # everything is done, reset state machine
                
        elif (self.state == "moveToNavigationAgain"):
            goal = graspingGoal()
            goal.object.function = "navigation"
            
            self.grasping.send_goal(goal)
            
            self.grasping.wait_for_result(rospy.Duration.from_sec(40.0)) # time-out checking, if takes to long stop waiting... shouldn't take longer then 1 minute?

            if (self.grasping.get_state() == actionlib.GoalStatus.SUCCEEDED):
                    time.sleep(9) # again, living dangerously
                    try:
                        self.service_write(rospy.Time.now(), "putDown", cjson.encode("True"))
                    except:
                        self.reinit() 
                        
                    self.state = "waitingForObject"  # reset the state machine
                    print "move to navigation is done"
                    

        signal.alarm(interval)

def main(args):
    rospy.init_node('object_locator_client')
    example = ActionClient()

    signal.signal(signal.SIGALRM, example.update)
    signal.alarm(interval)


    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)
