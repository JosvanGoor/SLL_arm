#!/usr/bin/env python

from sensor_msgs.msg import *
from std_msgs.msg import *
from basic_exercises.srv import *
import rospy

def convert_rgb_2_grey (data):
    img=data.img;
    res=Image()
    res.header=img.header;
    res.height=img.height;
    res.width=img.width;
    res.encoding = "mono8";
    res.is_bigendian = img.is_bigendian;
    res.step=res.width;
    res.data= [];

    b=bytearray();
    b.extend(img.data);

    print "callback called"
    for i in range(0, res.height*res.width):
        start=i*3;
        res.data.append(0.2989*b[start]+0.5870*b[start+1]+0.1140*b[start+2]);

    print "returned call"
    return res;

def rgb_server():
    rospy.init_node('rgb_server');
    s = rospy.Service('rgb2grey',rgb2grey,convert_rgb_2_grey);
    print "ready to convert";
    rospy.spin();

if __name__ == "__main__":
    rgb_server();


