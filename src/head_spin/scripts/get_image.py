import math, numpy, time, cv2
import rospy, roslib
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
import tf

from os.path import join
import os

###########

def convert_image(data):
    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")  
    except CvBridgeError, e:
      print e

    cv_image = numpy.asarray(cv_image)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(1)

    return cv_image

def take_image(take):
    global make_image
    
    if (take.data == True):
        make_image = True


def receive_image(image):
    global make_image
    global hsv_data
    global label
    global count
    
    if (make_image == True):
        image = convert_image(image)
        
        
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        image_hsv = cv2.resize(image_hsv, hsv_shape, interpolation = cv2.INTER_AREA)
        image_hsv = numpy.asarray(image_hsv, dtype=numpy.float32)
        image_hsv[:,:,0] /= 179.
        image_hsv[:,:,1] /= 255.
        image_hsv[:,:,2] /= 255.

        hsv_data.append(numpy.reshape(image_hsv, hsv_flat_shape))
        now = rospy.Time(0)
        try:
            transformer.waitForTransform("/map", "/front_xtion_link", now, rospy.Duration(0.01))
        except tf.Exception as e:
            print repr(e)
            return
        
        translation, rotation = transformer.lookupTransform("map", "front_xtion_link", now)
        
        orientation = tf.transformations.euler_from_quaternion(rotation)
    
        new_theta = orientation[2] / (math.pi / 180.0)
        
        odom = {'x': translation[0], 
                     'y': translation[1],
                     'angle': new_theta,
                     'quaternion': rotation,
                     'battery_level': 13.0,
                     'time' : time.time()}
        
        cur_odom = numpy.asarray([odom['x'], odom['y'], math.sin(odom['angle']), math.cos(odom['angle'])])


        label.append(cur_odom)
        
        #image has been taken
        make_image = False
        imageTakenPub.publish(True)
        
        count += 1

        if count % 200 == 0:
            print count
            

if __name__ == "__main__":
    
    path = os.getenv("HOME") + '/nav-data/realImageData/'
    
    if not os.path.exists(path): 
        os.makedirs(path)
    
    rospy.init_node('image_data_gatherer', anonymous=True)
    rospy.loginfo("Node initialized")
    
    bridge = CvBridge()
    
    hsv_shape = shape = (28, 28)
    hsv_flat_shape = (1, 28 * 28 * 3)
    
    imageSub = rospy.Subscriber('/front_xtion/rgb/image_raw', Image, receive_image)
    
    imageReadySub = rospy.Subscriber("headSpinner/getImage", Bool, take_image)
    imageTakenPub = rospy.Publisher("headSpinner/imageReceived", Bool, queue_size=1)
    
    transformer = tf.TransformListener()
    
    hsv_data = [] 
    label = []
    count = 0
    make_image = False
    
    rospy.loginfo("Callbacks registered")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
        

    print 'Number of images: ', count
    #saving images
    hsv_data = numpy.vstack(hsv_data)
    label = numpy.vstack(label)
    #print 'Not saving...........................!!!!!!!'
    timeStamp = time.strftime("%c") 
    numpy.save(join(path, "realHSV " + timeStamp + ".npy"), hsv_data)
    numpy.save(join(path, "label_realHSV " + timeStamp + ".npy"), label)
    
