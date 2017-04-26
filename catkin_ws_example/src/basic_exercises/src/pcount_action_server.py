#! /usr/bin/env python

import rospy

import actionlib
from sensor_msgs.msg import *
import basic_exercises.msg

class PCAction(object):
   _feedback = basic_exercises.msg.pixelcountFeedback();
   _result = basic_exercises.msg.pixelcountResult();

   def __init__(self, name):
      self._action_name = name
      self._as = actionlib.SimpleActionServer(self._action_name, basic_exercises.msg.pixelcountAction, execute_cb=self.execute_cb, auto_start = False)
      self._as.start()

   def execute_cb(self, goal):
      img=goal.grey;
      pixelCount=img.height*img.width*4;
      self._result.count=pixelCount;
      rospy.loginfo('%s: Succeeded' % self._action_name);
      self._as.set_succeeded(self._result);
      
      
if __name__ == '__main__':
   rospy.init_node('pixelCount');
   server = PCAction(rospy.get_name());
   rospy.spin();