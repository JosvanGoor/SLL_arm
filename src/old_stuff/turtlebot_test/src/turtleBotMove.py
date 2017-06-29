import rospy
import roslib
import sys
import thread

from geometry_msgs.msg import Twist


class TurtleBotMove:
    def __init__(self):
        rospy.init_node('move')
        self.p = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)
        self.x_speed = 0.0# 0.1 m/s
        thread.start_new_thread(self.update, ())
        self.max_speed = 0.1

    def moveForward(self):
        print 'setting speed'
        self.x_speed = self.max_speed

    def stop(self):
        print 'stopping'
        self.x_speed = 0.0

    def moveBackward(self):
        print 'moving backwards'
        self.x_speed = -self.max_speed;


    def update(self):

        r = rospy.Rate(100) 

        twist = Twist()

        while not rospy.is_shutdown():
            twist.linear.x  = self.x_speed;
            twist.linear.y = 0
            twist.linear.z = 0

            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0

       #     self.p.publish(twist)
            
            r.sleep()
