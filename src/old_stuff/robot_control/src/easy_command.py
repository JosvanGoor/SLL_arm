#!/usr/bin/env python
import numpy
import math

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, PoseWithCovarianceStamped, Twist
from borg_pioneer.srv import *
import borg_pioneer
import tf
import cjason

def controller():
    try:
        last_action_time = 0
        while not rospy.is_shutdown():
            rospy.sleep(1.0)
            resp = service_read('get_last_observation',rospy.Time.now(),'navigation_action','')
            resp = cjson.decode(resp.json)

            if not resp == None:
                if not resp['action_time'] == last_action_time:
                    last_action_time = resp['action_time']
                    rospy.loginfo(resp['command'])
                    command = resp['command']
                    param = resp['parameters']
                    if command == "move":
                        move(param)
                    else:
                        turn(param)
                    
                    rospy.sleep(1.0)

    except rospy.ServiceException, e:
        print "Service did not process request: %s"% str(e)

def move(target_range):
    listener.waitForTransform("base_link", "odom", rospy.Time(), rospy.Duration(1))
    
    try:
        start_loc, start_quat = listener.lookupTransform('base_link', 'odom', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException):
        continue
    
    base_cmd = Twist()
    #the command will be to go forward at 0.25 m/s
    base_cmd.linear.y = base_cmd.angular.z = 0;
    base_cmd.linear.x = 0.25;
    
    rate = rospy.Rate(20.0)
    
    done = False
    while not done:
        publisher.publish(base_cmd)
        rate.sleep()
        
        try:
            current_loc, current_quat = listener.lookupTransform('base_link', 'odom', rospy.Time(0))
        except tf.Exception as ex:
            rospy.logdebug(ex.what)
            break
        
        dist_moved = math.sqrt(numpy.dot(numpy.array(start_loc), numpy.transpose(current_loc)))
        
        if dist_moved > target_range:
            done = True
    return done

def turn(clockwise, radians):
    while radians < 0:
        radians += 2 * M_PI;
    while radians > 2* M_PI:
        radians -= 2 * M_PI;
    
    listener.waitForTransform("base_link", "odom", rospy.Time(), rospy.Duration(1))
    
    try:
        start_loc, start_quat = listener.lookupTransform('base_link', 'odom', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException):
        continue
    
    base_cmd.linear.x = base_cmd.linear.y = 0.0;
    base_cmd.angular.z = 0.75;
    if clockwise:
        base_cmd.angular.z = -base_cmd.angular.z;
    
    desired_turn_axis = (0,0,1)
    if clockwise:
        desired_turn_axis = -desired_turn_axis
if __name__ == '__main__':
    try:
        rospy.init_node('easy_command', anonymous=True)
        
        publisher = rospy.Publisher('cmd_vel', Twist)
        listener = tf.TransformListener()
        while True:
            print "Initializing memory_client for movement command"

            rospy.loginfo("memory_reader started")

            service_read = rospy.ServiceProxy('memory_read', MemoryReadSrv)
            rospy.wait_for_service('memory_read')
            rospy.loginfo("Read memory service")
            
            controller()

    except rospy.ROSInterruptException:
        pass
