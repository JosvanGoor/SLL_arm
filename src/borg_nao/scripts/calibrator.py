#!/usr/bin/env python

import rospy
import cv2
from editor import Editor

import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class GNUI:

    def __init__(self):
        self.editor = Editor("../launch/calibrated.launch")
      
        rospy.Subscriber("/NAO/image_bottom", Image, self.callback) 
        self.bridge = CvBridge()
        
        #Boundaries
        self.lower = np.array([0,0,0], dtype=np.uint8)
        self.upper = np.array([255,255,255], dtype=np.uint8)
         
        #GNUI
        cv2.namedWindow("Calibrator")
        cv2.createTrackbar("H upper","Calibrator",0, 255, self.H_upper)
        cv2.setTrackbarPos("H upper","Calibrator", 255)
        cv2.createTrackbar("H lower","Calibrator",0, 255, self.H_lower)
        
        
        cv2.createTrackbar("S upper","Calibrator",0, 255, self.S_upper)
        cv2.setTrackbarPos("S upper","Calibrator",255)
        cv2.createTrackbar("S lower","Calibrator",0, 255, self.S_lower)
    
        cv2.createTrackbar("V upper","Calibrator",0, 255, self.V_upper)
        cv2.createTrackbar("V lower","Calibrator",0, 255, self.V_lower)
        cv2.setTrackbarPos("V upper","Calibrator",255)
        
        
        
    
    def exportRed(self):           
      self.editor.setRed(self.upper[0], self.lower[0], self.upper[1], self.lower[1], self.upper[2], self.lower[2])
      self.editor.writeParams()
      print "exported red"
      
      
    def exportBlue(self):
      self.editor.setBlue(self.upper[0], self.lower[0], self.upper[1], self.lower[1], self.upper[2], self.lower[2])
      self.editor.writeParams()
      print "exported blue"
      
    def exportYellow(self):
      self.editor.setYellow(self.upper[0], self.lower[0], self.upper[1], self.lower[1], self.upper[2], self.lower[2])
      self.editor.writeParams()
      print "exported yellow"
      
    def importRed(self):
      self.upper[0], self.lower[0], self.upper[1], self.lower[1], self.upper[2], self.lower[2] = self.editor.getRed()
      cv2.setTrackbarPos("V upper","Calibrator", self.upper[2])
      cv2.setTrackbarPos("V lower","Calibrator", self.lower[2])
      cv2.setTrackbarPos("S upper","Calibrator",self.upper[1])
      cv2.setTrackbarPos("S lower","Calibrator", self.lower[1])
      cv2.setTrackbarPos("H upper","Calibrator", self.upper[0])
      cv2.setTrackbarPos("H lower","Calibrator", self.lower[0])

      
    def importYellow(self):
      self.upper[0], self.lower[0], self.upper[1], self.lower[1], self.upper[2], self.lower[2] = self.editor.getYellow()
      cv2.setTrackbarPos("V upper","Calibrator",self.upper[2])
      cv2.setTrackbarPos("V lower","Calibrator", self.lower[2])
      cv2.setTrackbarPos("S upper","Calibrator",self.upper[1])
      cv2.setTrackbarPos("S lower","Calibrator", self.lower[1])
      cv2.setTrackbarPos("H upper","Calibrator", self.upper[0])
      cv2.setTrackbarPos("H lower","Calibrator", self.lower[0])
  
      
    def importBlue(self):
      self.upper[0], self.lower[0], self.upper[1], self.lower[1], self.upper[2], self.lower[2] = self.editor.getBlue()
      cv2.setTrackbarPos("V upper","Calibrator",self.upper[2])
      cv2.setTrackbarPos("V lower","Calibrator", self.lower[2])
      cv2.setTrackbarPos("S upper","Calibrator",self.upper[1])
      cv2.setTrackbarPos("S lower","Calibrator", self.lower[1])
      cv2.setTrackbarPos("H upper","Calibrator", self.upper[0])
      cv2.setTrackbarPos("H lower","Calibrator", self.lower[0])
      
    def updateRos(self):
        return 0
      
    def callback(self, data):
        temp = self.bridge.imgmsg_to_cv2(data)
        image  = np.asarray(temp[:,:]) 
        image = cv2.blur(image, (5,5))
        hsv_img = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv_img, self.lower, self.upper)
        res = cv2.bitwise_and(image, image, mask = mask)


        cv2.imshow("Calibrator", res)
        ##cv2.imshow("Calibrator", mask)
        key = cv2.waitKey(10)
        if key != -1:
	   print key
           
        if key == 1114194: ## This is shift + R
           self.exportRed()
        if key == 1114178: ## This is shift + B
           self.exportBlue()
        if key == 1114201: ## This is shift + Y
           self.exportYellow()
           
        if key == 1310841:
           print "importing yellow..."
           self.importYellow() ##ctrl + y
           
        if key == 1310834:
           print "importing red..."
           self.importRed()  ##ctrl + r
           
        if key == 1310818:
           print "importing blue..."
           self.importBlue()  ##ctrl + b

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

    rospy.init_node("colorblob_calibrator")
    rospy.spin()
