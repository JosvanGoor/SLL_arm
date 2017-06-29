import rospy
import actionlib
from moveit_msgs.msg import * 
from control_msgs.msg import *

class SimulationController(object):
    
    def __init__(self):
        self._as = actionlib.SimpleActionServer("/mico_arm/joint_velocity_controller/trajectory", FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False);
        self._as.start()
        
    
    def execute_cb(self, goal):
               
        
        
        

if __name__ == "__main__":
    rospy.init_node("simulation_server");
    SimulationController();
    rospy.spin();
    
    