#!/usr/bin/python

import cv2 as cv
import numpy as np
import rospy
import math
import sys

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class VisionSystem(object):
    def __init__(self, min_thresh=227, max_tresh=255):
        self.subscriber = rospy.Subscriber("usb_cam/image_raw", Image, self.callback, queue_size = 1)

        self.publisher = rospy.Publisher("/detectedcontours/image_raw", Image)

        self.bridge = CvBridge()


    def callback(self, ros_data):

        cv_image = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")

        (height, width, channels) = cv_image.shape

        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)

        n, threshImg = cv.threshold(gray, 250, 255, cv.THRESH_BINARY)

        n, contours, h = cv.findContours(threshImg, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
  
        cv.drawContours(cv_image, contours, -1, (0,255,0), 3)

        self.publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))



def main(args):
    vision = VisionSystem()
    rospy.init_node('VisionSystem', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS vision system"
    
if __name__ == '__main__':
    main(sys.argv)
