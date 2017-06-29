import rospy
from std_msgs.msg import Float64
import math

def sendVelocity():
    pub1.publish(math.radians(0))
    pub2.publish(math.radians(0))
    pub3.publish(math.radians(0))
    pub4.publish(math.radians(0))
    pub5.publish(math.radians(0))
    pub6.publish(math.radians(0))
    
    

if __name__=="__main__":
    rospy.init_node('velocity_sender', anonymous=True)
    rate = rospy.Rate(10)
    rate.sleep()
    
    pub1 = rospy.Publisher('/mico/joint1_controller/command', Float64, queue_size=1)
    '''
    pub2 = rospy.Publisher('/mico/joint2_controller/command', Float64, queue_size=1)
    pub3 = rospy.Publisher('/mico/joint3_controller/command', Float64, queue_size=1)
    pub4 = rospy.Publisher('/mico/joint4_controller/command', Float64, queue_size=1)
    pub5 = rospy.Publisher('/mico/joint5_controller/command', Float64, queue_size=1)
    pub6 = rospy.Publisher('/mico/joint6_controller/command', Float64, queue_size=1)
    '''
    for i in range(0, 50):
        rate.sleep()
        sendVelocity()
    
    rospy.spin()
