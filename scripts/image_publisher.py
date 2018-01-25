#!/usr/bin/env python

import roslib
#roslib.load_manifest('image')
import sys,rospy,cv2
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image,CompressedImage
import numpy as np
import threading, time


class image_publisher:

    def __init__(self):
        #super(image_publisher, self).__init__()
        #rospy.init_node('save_image')
        self.term = False
        self.imglock = threading.Lock()
        #cv2.namedWindow("test", 1)

        self.img = None
        self.bridge = CvBridge()
        #self.cam = rospy.Subscriber("/camera/rgb/image_rect_color/compressed",CompressedImage,self.callback) 
        self.cam = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        #self.cam = rospy.Subscriber("/stereo/right/camera/image/compressed",CompressedImage,self.callback) 
        self.secs = 0
        self.nsecs = 0
        self.image = None

    def callback(self,data):
        self.imglock.acquire()
        self.stamp = data.header.stamp

        np_arr = np.fromstring(data.data, dtype=np.uint8)
        self.img = cv2.imdecode(np_arr,1) # compressed image

        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e
        '''try:
      	    cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            #size = cv_image.shape[:2]
            #print size
            #grey = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
   	    #grey = cv2.blur(grey,(7,7))
      
            #cv2.imshow("test", grey)
            cv2.imshow("test",cv_image)
            cv2.waitKey(3)
        except CvBridgeError, e:
            print e
        '''
        #cv2.destroyAllWindows()
        
        self.imglock.release()
    
    def get_img(self):
        #if self.img == None:
	#    return (None, -1)
        if self.image == None:
             return (None, -1)
	self.imglock.acquire()
	rimg = np.copy(self.img)
	self.imglock.release()
        return (self.image,self.stamp)
        #return (rimg,self.stamp)
