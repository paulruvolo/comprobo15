#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS. """

import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class BlobDetector(object):
    """ The BlobDetector is a Python object that encompasses a ROS node 
        that can process images from the camera and search for blobs within """
    def __init__(self, image_topic):
        """ Initialize the blob detector """
        rospy.init_node('blob_detector')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        cv2.namedWindow('camera_image')             # a window for the latest camera image
        #cv2.setMouseCallback('camera_image', self.process_mouse_event)
        rospy.Subscriber(image_topic, Image, self.process_image)
        cv2.namedWindow('image_info')               # a window to show color values of pixels
        cv2.namedWindow('tracking_window')
        self.param_1 = 25
        self.param_2 = 25
        self.param_3 = 25
        cv2.createTrackbar('param1', 'tracking_window', 0, 255, self.set_param_1)
        cv2.createTrackbar('param2', 'tracking_window', 0, 255, self.set_param_2)
        cv2.createTrackbar('param3', 'tracking_window', 0, 255, self.set_param_3)

    def set_param_1(self, val):
        self.param_1 = val

    def set_param_2(self, val):
        self.param_2 = val

    def set_param_3(self, val):
        self.param_3 = val

    def detect(self):
        """ Search for a blob in the last image found """
        if self.cv_image == None:
            return
        my_image = deepcopy(self.cv_image)
        my_image_gray = cv2.cvtColor(my_image, cv2.COLOR_BGR2GRAY)
        my_image_gray = cv2.bilateralFilter(my_image_gray,self.param_1,self.param_2,self.param_3)

        ret, thresholded = cv2.threshold(my_image_gray, 5, 255, cv2.THRESH_BINARY)
        # for i in range(my_image.shape[0]):
        #     for j in range(my_image.shape[1]):
        #         # process pixels here
        #         pass
        #average = np.mean(thresholded,axis=0).reshape((1,640))
        #average = (np.tile(average,(my_image.shape[0],1))*255.0).astype('uint8')
        print thresholded.shape
        visualize = cv2.cvtColor(thresholded, cv2.COLOR_GRAY2BGR)

        lines = cv2.HoughLinesP(thresholded,1,np.pi/180,200, minLineLength=100)
        print lines.shape
        # for  x1,y1,x2,y2 in lines[0]:
        #     cv2.line(visualize,(x1,y1),(x2,y2),(0,255,0),1)
        # draw a red circle at the center of your blob
        #cv2.circle(thresholded, (int(0), int(0)), 5, (0,0,255),-1)
        print "recalculating"
        cv2.imshow('tracking_window', visualize)
        cv2.waitKey(20)

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow("camera_image", self.cv_image)
        cv2.waitKey(5)

    def process_mouse_event(self, event, x,y,flags,param):
        """ Process mouse events so that you can see the color values associated
            with a particular pixel in the camera images """
        if event == cv2.EVENT_LBUTTONDOWN:
            self.detect()
        image_info_window = 255*np.ones((500,500,3))
        cv2.putText(image_info_window, 'Color (b=%d,g=%d,r=%d)' % (self.cv_image[y,x,0], self.cv_image[y,x,1], self.cv_image[y,x,2]), (5,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0))
        cv2.imshow('image_info', image_info_window)
        cv2.waitKey(20)

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        iter_count = 0
        while not rospy.is_shutdown():
            #self.detect()
            if iter_count % 3 == 0:
                t = time.time()
                self.detect()
                print time.time() - t
            iter_count += 1
            cv2.waitKey(20)
            r.sleep()

if __name__ == '__main__':
    node = BlobDetector("/camera/image_raw")
    node.run()