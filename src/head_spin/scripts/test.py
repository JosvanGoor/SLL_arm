import math, numpy, time, cv2
import rospy, roslib
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
import message_filters

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
import tf

from os.path import join
import os


def convert_image(data, type = "bgr8"): # use type = "16UC1" for depth/image_raw
    try:
      cv_image = bridge.imgmsg_to_cv2(data, type)  
    except CvBridgeError, e:
      print e

    
    
    cv_image = numpy.asarray(cv_image)
        
    
   # cv2.imshow("Image window", cv_image)
   # cv2.waitKey(1)

    return cv_image

def take_image(take):
    global make_image
    
    if (take.data == True):
        make_image = True


def receive_image(depthImage, rgbImage):
    
    depthImage = convert_image(depthImage, type="16UC1")
   
    #MAX = 9870
    #MIN = 350
    depthImage = numpy.asarray(depthImage, dtype = numpy.float32)
    depthImage /= 9870.   
        
    cv2.imshow("Depth image", depthImage)
    
    rgbImage = convert_image(rgbImage)
    cv2.imshow("RGB image", rgbImage)
    cv2.waitKey(1)
    
    
                

if __name__ == "__main__":
       
    rospy.init_node('image_data_gatherer', anonymous=True)
    rospy.loginfo("Node initialized")
    
    bridge = CvBridge()
    
    hsv_shape = shape = (28, 28)
    hsv_flat_shape = (1, 28 * 28 * 3)
    
    depthSub = message_filters.Subscriber('/camera/depth/image_raw', Image)
    imageSub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
    
    ts = message_filters.ApproximateTimeSynchronizer([depthSub, imageSub], 1, 0.05)
    ts.registerCallback(receive_image)
    
    
    hsv_data = [] 
    label = []
    count = 0
  #  make_image = False
    
    rospy.loginfo("Callbacks registered")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
        
    '''
    print 'Number of images: ', count
    #saving images
    hsv_data = numpy.vstack(hsv_data)
    label = numpy.vstack(label)
    #print 'Not saving...........................!!!!!!!'
    timeStamp = time.strftime("%c") 
    numpy.save(join(path, "realHSV " + timeStamp + ".npy"), hsv_data)
    numpy.save(join(path, "label_realHSV " + timeStamp + ".npy"), label)
    '''
