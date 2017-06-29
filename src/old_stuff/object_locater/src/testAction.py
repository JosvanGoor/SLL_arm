import roslib; roslib.load_manifest('object_locater'); roslib.load_manifest('jaco_msgs')
import rospy
import actionlib

from object_locater.msg import *
from jaco_msgs.msg import *

if __name__ == '__main__':

    rospy.init_node('object_locator_client')

 #   client = actionlib.SimpleActionClient('locate_object', locateObjectAction)
 #   print 'Waiting for locate object server'
 #   client.wait_for_server()

    grasping = actionlib.SimpleActionClient("grasping", graspingAction)
    print 'waiting for grasping server'
    grasping.wait_for_server()

    print 'Starting' 
    '''
    goal = locateObjectGoal()
    goal.startLocating = True
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(20.0))  # use this for time-out checking, if takes to long stop waiting
    result = None
    
    grasp = False
    drop = False
    
    if (client.get_state() == actionlib.GoalStatus.ABORTED):  # Could not find object after trying 10 times
    	print 'aborted!' 
    	result = client.get_result()
    	print 'Result: ', result.object.correct

    elif (client.get_state() == actionlib.GoalStatus.SUCCEEDED):  # has found one cluster and is ready for grasping
        print 'Succes!'
        result = client.get_result()
    	grasp = True
    	print 'Result:', result.object.correct 

    elif (client.get_state() != actionlib.GoalStatus.SUCCEEDED): # most likely the time-out accord
    	print 'Time out' 
    	
    '''
    grasp = True
    if (grasp == True):
        goal = graspingGoal()
        '''
        goal.object.function = "home"
        
        grasping.send_goal(goal)
        
        grasping.wait_for_result(rospy.Duration.from_sec(60.0)) # time-out
        
        if (grasping.get_state() == actionlib.GoalStatus.SUCCEEDED):
            print 'moved to home position'
        
        else:
            print 'moved to home FAILED'
        '''    
        goal.object.function = "navigation"
        
        grasping.send_goal(goal)
        
        grasping.wait_for_result(rospy.Duration.from_sec(60.0)) 
        
        if (grasping.get_state() == actionlib.GoalStatus.SUCCEEDED):
            print 'moved to navigation position'
        else:
            print 'moving to navigation FAILED'
        
        '''    
        goal.object.function = "fromNavToHome"
        
        grasping.send_goal(goal)
        
        grasping.wait_for_result(rospy.Duration.from_sec(60.0)) 
        
        if (grasping.get_state() == actionlib.GoalStatus.SUCCEEDED):
            print 'moved to home position'
        else:
            print 'moving to home FAILED'
        
        '''
        
        '''
        print 'Grasping part' 

        goal = graspingGoal()
        goal.object.x = result.object.x
        goal.object.y = result.object.y
        goal.object.z = result.object.z 
        goal.object.thetaX = result.object.thetaX
        goal.object.thetaY = result.object.thetaY
        goal.object.thetaZ = result.object.thetaZ
        goal.object.height = result.object.height

        print 'Going to grasp object'
        goal.object.function = "grasp"
        
        grasping.send_goal(goal)

        grasping.wait_for_result(rospy.Duration.from_sec(60.0)) # time-out checking, if takes to long stop waiting... shouldn't take longer then 1 minute?

        if (grasping.get_state() == actionlib.GoalStatus.ABORTED):
            result = grasping.get_result()
            print 'Aborted, something went wrong, moving to home position'
            goal.object.function = "home"; # send it to home position

            grasping.send_goal(goal)
            grasping.wait_for_result(rospy.Duration.from_sec(60.0))

        elif (grasping.get_state() == actionlib.GoalStatus.SUCCEEDED):
            print 'done grasping'
            grasp == False
            drop = True

        elif (grasping.get_state() != actionlib.GoalStatus.SUCCEEDED):
        	print 'time-out'

    if (drop == True):
        print 'Dropping part'

        goal = locateObjectGoal()
        goal.startLocating = True
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(20.0))  # use this for time-out checking, if takes to long stop waiting
        result = None

        if (client.get_state() == actionlib.GoalStatus.ABORTED):  # Could not find object after trying 10 times
            print 'aborted!' 
            result = client.get_result()
            print 'Result: ', result.object.correct

        elif (client.get_state() == actionlib.GoalStatus.SUCCEEDED):  # has found one cluster and is ready for grasping
            print 'Succes!'
            result = client.get_result()

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

            grasping.send_goal(goal)

            grasping.wait_for_result(rospy.Duration.from_sec(60.0)) # time-out checking, if takes to long stop waiting... shouldn't take longer then 1 minute?

            if (grasping.get_state() == actionlib.GoalStatus.ABORTED):
                result = grasping.get_result()
                print 'Aborted, something went wrong, moving to home position'
                goal.object.function = "home"; # send it to home position

                grasping.send_goal(goal)
                grasping.wait_for_result(rospy.Duration.from_sec(60.0))

            elif (grasping.get_state() == actionlib.GoalStatus.SUCCEEDED):
                print 'done dropping'

        elif (client.get_state() != actionlib.GoalStatus.SUCCEEDED): # most likely the time-out accord
            print 'Time out' 

        
        '''
    print 'done'
