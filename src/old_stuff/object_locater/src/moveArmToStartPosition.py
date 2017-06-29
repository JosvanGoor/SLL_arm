import roslib; roslib.load_manifest('object_locater'); roslib.load_manifest('jaco_msgs')
import rospy
import actionlib

from object_locater.msg import *
from jaco_msgs.msg import *

if __name__ == '__main__':
    rospy.init_node('simple_init_client')

    grasping = actionlib.SimpleActionClient("grasping", graspingAction)
    print 'waiting for grasping server'
    grasping.wait_for_server()
    
    goal = graspingGoal()
    
    goal.object.function = "navigation"
        
    grasping.send_goal(goal)
    
    grasping.wait_for_result(rospy.Duration.from_sec(60.0)) 
    
    if (grasping.get_state() == actionlib.GoalStatus.SUCCEEDED):
        print 'moved to navigation position'
    else:
        print 'moving to navigation FAILED'