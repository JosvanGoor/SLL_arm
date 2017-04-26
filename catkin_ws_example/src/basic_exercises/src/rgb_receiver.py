#!/usr/bin/env python

from sensor_msgs.msg import Image
from std_msgs.msg import *
from basic_exercises.srv import *
from basic_exercises.msg import *
import rospy
import actionlib

__client = actionlib.SimpleActionClient('pixelCount', basic_exercises.msg.pixelcountAction)
__pub = rospy.Publisher('final', resultbag, queue_size=10)

def callback(data):

	print "callback called"
	rospy.wait_for_service('rgb2grey');
	img=data;
	try:
		rgb2greyFunction = rospy.ServiceProxy('rgb2grey',rgb2grey)
		grey=rgb2greyFunction(img).res;
		print "grey received \n"
		__client.wait_for_server();

		print "Pixel counter ready"
		goal = basic_exercises.msg.pixelcountGoal(img,grey);


		print "send command pixel counter"
		__client.send_goal(goal);
		print "wait for answer"
		__client.wait_for_result();

		count=__client.get_result().count;
		print "pixel count received\n"

		__pub.publish(img,grey,count);
		print "published result"
	except rospy.ServiceException, e:
		print "Service call failed: %s" %e


def rgb_receiver():

   # In ROS, nodes are uniquely named. If two nodes with the same
   # node are launched, the previous one is kicked off. The
   # anonymous=True flag means that rospy will choose a unique
   # name for our 'listener' node so that multiple listeners can
   # run simultaneously.
   rospy.init_node('rgb_receiver', anonymous=True)
   rospy.Subscriber("/camera/rgb/image_raw", Image, callback)
   # spin() simply keeps python from exiting until this node is stopped
   rospy.spin()

if __name__ == '__main__':
   rgb_receiver()