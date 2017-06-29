#!/usr/bin/env python

import rospy
import cv2

import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
class GNUI:
    
    def __init__(self):
        rospy.Subscriber("/NAO/image_bottom", Image, self.callback) 
        self.bridge = CvBridge()
        
        #Boundaries
        self.lower = np.array([0,0,0], dtype=np.uint8)
        self.upper = np.array([255,255,255], dtype=np.uint8)
         
        #GNUI
        cv2.namedWindow("Calibrator")
        cv2.createTrackbar("H upper","Calibrator",0, 255, self.H_upper)
        cv2.setTrackbarPos("H upper","Calibrator",255)
        cv2.createTrackbar("H lower ","Calibrator",0, 255, self.H_lower)
    
        cv2.createTrackbar("S upper","Calibrator",0, 255, self.S_upper)
        cv2.setTrackbarPos("S upper","Calibrator",255)
        cv2.createTrackbar("S lower ","Calibrator",0, 255, self.S_lower)
    
        cv2.createTrackbar("V upper","Calibrator",0, 255, self.V_upper)
        cv2.createTrackbar("V lower ","Calibrator",0, 255, self.V_lower)
        cv2.setTrackbarPos("V upper","Calibrator",255)
    
    
    def callback(self, data):
        temp = self.bridge.imgmsg_to_cv2(data)
        image  = np.asarray(temp[:,:]) 
        image = cv2.blur(image, (5,5))
        hsv_img = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv_img, self.lower, self.upper)
        res = cv2.bitwise_and(image, image, mask = mask)
        im2, contours, hierarchy =         cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for points in countours:
          if(cv.contourArea(points) > )

        cv2.imshow("Calibrator", res)
        ##cv2.imshow("Calibrator", mask)
        cv2.waitKey(10)

    def H_upper(self, data):
        self.upper[0] = data
        
    def H_lower(self, data):
        self.lower[0] = data
        
    def S_upper(self, data):
        self.upper[1] = data
        
    def S_lower(self, data):
        self.lower[1] = data

    def V_upper(self, data):
        self.upper[2] = data
            
    def V_lower(self, data):
        self.lower[2] = data

if __name__ == '__main__':
    GUI =  GNUI()

    rospy.init_node("Colorblob Annotator")
    rospy.spin()