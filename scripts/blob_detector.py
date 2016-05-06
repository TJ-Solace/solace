#!/usr/bin/env python

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#import threading
from solace.msg import BlobDetections
from geometry_msgs.msg import Point
import math
import time
import sys


class BlobDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub_blobs = rospy.Publisher("/blobs", BlobDetections, queue_size=1)
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.processImage, queue_size=1)
        
    def processImage(self, image_msg):
        im = self.bridge.imgmsg_to_cv2(image_msg)
        #height, width = im.shape[:2]
        #im = im[50:int(height/2)-40, 0:width]
        im = im[:.4*len(im)]
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        self.msg = BlobDetections()
        self.find_color(im, "red", cv2.inRange(hsv, np.array([0, 140, 30]), np.array([10, 240, 150])))       # red
        self.find_color(im, "green", cv2.inRange(hsv, np.array([45, 110, 100]), np.array([65, 210, 150])))   # green
        #self.find_color(im, "green", cv2.inRange(hsv, np.array([50, .4*255, .15*255]), np.array([77, 255, 255])))  # green
        #self.find_color(im, "orange", cv2.inRange(hsv, np.array([4, 230, 140]), np.array([6, 255, 200])))   # green
        #self.find_color(im, "yellow", cv2.inRange(hsv, np.array([40, 150, 100]), np.array([50, 200, 175])))  # yellow
        self.find_color(im, "blue", cv2.inRange(hsv, np.array([100, 120, 15]), np.array([130, 180, 80])))   # blue
        #self.find_color(im, "pink", cv2.inRange(hsv, np.array([170, 210, 160]), np.array([180, 230, 190])))    # pink
        if len(self.msg.heights) > 0:
            self.pub_blobs.publish(self.msg)

    def find_color(self, passed_im, label_color, mask):
        im = passed_im.copy()
        contours = cv2.findContours(mask, cv2.cv.CV_RETR_TREE, cv2.cv.CV_CHAIN_APPROX_SIMPLE)[0]
        approx_contours = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < 400: 
                continue
            perim = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, .05*perim, True)
            if len(approx) == 4:
                approx_contours.append(approx)
                self.msg.areas.append(area)
                self.msg.colors.append(label_color)
                moments = cv2.moments(c)
                center = (int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00']))
                msg_loc = Point()
                msg_loc.x, msg_loc.y = float(center[0]) / len(im[0]), float(center[1]) / len(im)
                self.msg.locations.append(msg_loc)
                self.msg.heights.append(float((max(approx, key=lambda x: x[0][1])[0][1] - min(approx, key=lambda x: x[0][1])[0][1])) / len(im))
                cv2.putText(im, label_color, center, cv2.FONT_HERSHEY_PLAIN, 2, (100, 255, 100))
                print "Label color:  {}".format(label_color)


if __name__ == "__main__":
    rospy.init_node("BlobDetector")
    bd = BlobDetector()
    rospy.spin()
    cv2.destroyAllWindows()
