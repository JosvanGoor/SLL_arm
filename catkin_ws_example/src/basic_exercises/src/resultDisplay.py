#!/usr/bin/env python

from sensor_msgs.msg import Image
from std_msgs.msg import *
from basic_exercises.srv import *
from basic_exercises.msg import *
import rospy
import actionlib
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy

__client = actionlib.SimpleActionClient('pixelCount', basic_exercises.msg.pixelcountAction)
__pub = rospy.Publisher('final', resultbag, queue_size=10)

def callback(data):
   result=data;
   print "callback called"
   rgb_cv=convert_image_cv(result.img);
   grey_cv=convert_image_cv(result.grey);
   rospy.loginfo(rospy.get_caller_id() + "There were %d pixels in the images", result.count);
   cv2.imshow("RGB",rgb_cv);
   cv2.imshow("Grey",grey_cv);
   cv2.waitKey(1)


def rgb_receiver():

   # In ROS, nodes are uniquely named. If two nodes with the same
   # node are launched, the previous one is kicked off. The
   # anonymous=True flag means that rospy will choose a unique
   # name for our 'listener' node so that multiple listeners can
   # run simultaneously.
   rospy.init_node('resultDisplay', anonymous=True)

   rospy.Subscriber("/final", resultbag, callback)
   # spin() simply keeps python from exiting until this node is stopped
   rospy.spin()

def convert_image_cv(data, type = "bgr8"):
    # use type = "8UC1" for grayscale images
    try:
        cv_image = bridge.imgmsg_to_cv2(data, type)
        # use "bgr8" if its a color image
    except CvBridgeError, e:
        print e
    ### Show the image
    
    return cv_image
## Convert from opencv format to sensor_msgs format
def convert_image_ros(data, type = "bgr8"):
    # use type = "8UC1" for grayscale images
    try:
        ros_image = bridge.cv2_to_imgmsg(data, type)
    except CvBridgeError, e:
        print e
    return ros_image

if __name__ == '__main__':
   bridge = CvBridge()
   print "test"
   rgb_receiver()