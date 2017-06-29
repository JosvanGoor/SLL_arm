import rospy
import actionlib
from moveit_msgs.msg import *

def test():
    client = actionlib.SimpleActionClient('pickup', PickupAction)
    
    print 'waiting for server'
    client.wait_for_server()
    print 'found pickup server'
    
    goal = PickupGoal()
    
    goal.target_name = "test";
    goal.group_name = "mico_arm";
    goal.end_effector = "mico_link_hand"
    
    client.send_goal(goal);

if __name__ == "__main__":
    rospy.init_node("pickup_test");
    test()
    

