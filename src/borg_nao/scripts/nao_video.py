#!/usr/bin/env python

import rospy
import sys
import vision_definitions
import cv2
import numpy as np
import time

from sensor_msgs.msg import Image
from naoqi import ALProxy
from cv_bridge import CvBridge, CvBridgeError

class nao_video:

    def __init__(self, ip, port, fps, rotate):
    
       self.rotate = rotate
    
       if self.rotate:
               print "Rotate"
    
       try:
               self.videoProxy = ALProxy("ALVideoDevice", ip, port)
       except Exception,e:
               print "Could not create video proxy"
               print str(e)
               exit(1)
       
       if not self.rotate:
               print "switching to bottom camera"
               self.videoProxy.setActiveCamera(1)
               
       resolution = vision_definitions.kQVGA
       colorspace = 13 
       
       self.proxyID = self.videoProxy.subscribe("python_GWM", resolution, colorspace, fps)
       self.bridge = CvBridge()
       self.pub = rospy.Publisher('NAO/image_bottom', Image, queue_size=0)
       
        # Set the parameters for each camera
       self.setCameraParameters(0)
       self.setCameraParameters(1)
        
    def setCameraParameters(self, cameraIndex, autoExposure=False, exposure=170, gain=72, autoWhitebalance=False, whitebalance=-103, brightness=86, \
                            contrast=32, saturation=128):
        
        self.videoProxy.setParameter(cameraIndex, vision_definitions.kCameraAutoExpositionID, autoExposure) # disable autoExposure        
        self.videoProxy.setParameter(cameraIndex, vision_definitions.kCameraExposureID, exposure) # Exposure
        self.videoProxy.setParameter(cameraIndex, vision_definitions.kCameraGainID, gain) # Gain
        
        self.videoProxy.setParameter(cameraIndex, vision_definitions.kCameraAutoWhiteBalanceID, autoWhitebalance) # disable auto white balance
        self.videoProxy.setParameter(cameraIndex, vision_definitions.kCameraWhiteBalanceID, whitebalance) # disable auto white balance
        self.videoProxy.setParameter(cameraIndex, vision_definitions.kCameraBrightnessID, brightness) # disable auto white balance
        self.videoProxy.setParameter(cameraIndex, vision_definitions.kCameraContrastID, contrast) # disable auto white balance
        self.videoProxy.setParameter(cameraIndex, vision_definitions.kCameraSaturationID, saturation) # disable auto white balance
                

    def __del__(self):
       self.videoProxy.unsubscribe(self.proxyID)
    
    def update(self):
       imageData = self.videoProxy.getImageRemote(self.proxyID)
    
       self.videoProxy.releaseImage(self.proxyID)
       
       width = imageData[0]
       height = imageData[1]
       channels = imageData[2]
       img = np.fromstring(imageData[6], dtype=np.uint8).reshape(height, width, channels)
    
       if self.rotate:
               img = cv2.transpose(img)
               img = cv2.flip(img,1) #Flip horizontal
    
       try:
               self.pub.publish(self.bridge.cv2_to_imgmsg(img))
       except CvBridgeError,e:
               print e
               
               #cv2.imshow("test",img)
               #cv2.waitKey(10)

def main(args):
       rospy.init_node('borg_nao_video')
       
       fps = rospy.get_param("~fps",30)
       ip = rospy.get_param("~nao_ip")
       port = rospy.get_param("~nao_port")
       rotate = rospy.get_param("~rotate")

       if (fps > 30):
               fps = 30

       r = rospy.Rate(fps)

       videoController = nao_video(ip, port, fps, rotate)

       while not rospy.is_shutdown():
               videoController.update()         
               r.sleep()

if __name__ == '__main__':
       main(sys.argv)
 