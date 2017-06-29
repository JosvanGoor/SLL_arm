import math, numpy, time, cv2
import rospy, roslib
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from cv_bridge import CvBridge, CvBridgeError

from os.path import join


path = '/home/rik/nav-data/realImageData/realHSV.npy'
images = numpy.load(path)

print len(images)

for image in images:
   
    print image.shape
    image = numpy.reshape(image, (28, 28, 3))
    image = cv2.resize(image, (280,280), interpolation = cv2.INTER_AREA)
   
    image[:,:,0] *= 179.
    image[:,:,1] *= 255.
    image[:,:,2] *= 255.
    
    image = numpy.asarray(image, dtype = numpy.uint8)
    
    image = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)
   
       
    
    
    cv2.imshow("Image window", image)
    cv2.waitKey(0)