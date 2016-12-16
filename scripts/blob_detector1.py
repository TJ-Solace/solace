#!/usr/bin/env python

import numpy as np
import cv2
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#from racecar_wk3.msg import BlobDetections
from std_msgs.msg import String
#from geometry_msgs.msg import Point

import math
import time
import sys


class BlobDetector:
    def __init__(self):
        self.isTesting = False
        self.bridge = CvBridge()
        self.pub_blobs = rospy.Publisher("/exploring_challenge", String, queue_size=1)
        self.sub_image = rospy.Subscriber("/zed/rgb/image_rect_color", Image, self.processImage, queue_size=1)
        print "Initialized blob detector"
        
    def processImage(self, image_msg):
        im = self.bridge.imgmsg_to_cv2(image_msg)
        #im = im[len(im)*.4:]
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        if not self.isTesting:
            self.find_color(im, "red", cv2.inRange(hsv, np.array([0, 130, 50]), np.array([10, 255, 140])))     # red
            self.find_color(im, "green", cv2.inRange(hsv, np.array([40, 100, 40]), np.array([85, 255, 200])))  # green
            self.find_color(im, "yellow", cv2.inRange(hsv, np.array([20, 165, 80]), np.array([40, 200, 120])))  # yellow
            self.find_color(im, "blue", cv2.inRange(hsv, np.array([100, 115, 55]), np.array([130, 255, 255])))  # blue
        else:
        	pass
            #self.find_color(im, "testing",cv2.inRange(hsv, np.array([self.hl, self.sl, self.vl]), np.array([self.hu, self.su, self.vu])))
            #self.challenge_color(im, "challenge testing", cv2.inRange(hsv, np.array([self.hl, self.sl, self.vl]), np.array([self.hu, self.su, self.vu])))  # pink
    def find_color(self, passed_im, label_color, mask):
    	im = passed_im.copy()
        if self.isTesting:
            self.image = im
        contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[1]
        approx_contours = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < 500: 
                continue
            perim = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, .02*perim, True)
            if len(approx) == 4:  # rectangle
            	approx_contours.append(approx)
                rect_msg = String()
                rect_msg.data = "{} rectangle".format(label_color)
                self.pub_blobs.publish(rect_msg)
                moments = cv2.moments(c)
                center = (int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00']))
                im = passed_im.copy()
                cv2.putText(im, "{} rectangle".format(label_color), center, cv2.FONT_HERSHEY_PLAIN, 2, (100, 255, 100))
                cv2.drawContours(im, [approx], -1, (100, 255, 100), 2)
                cv2.imwrite("/home/ubuntu/challenge_photos1/{}rectangle{}.png".format(label_color, int(time.clock()*1000)), im)
                print "{}square".format(label_color)
            elif abs(len(approx)-12) <= 1:  # cross
            	approx_contours.append(approx)
                cross_msg = String()
                cross_msg.data = "{} cross".format(label_color)
                self.pub_blobs.publish(cross_msg)
                moments = cv2.moments(c)
                center = (int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00']))
                im = passed_im.copy()
                cv2.putText(im, "{} cross".format(label_color), center, cv2.FONT_HERSHEY_PLAIN, 2, (100, 255, 100))
                cv2.drawContours(im, [approx], -1, (100, 255, 100), 2)
                cv2.imwrite("/home/ubuntu/challenge_photos1/{}cross{}.png".format(label_color, int(time.clock()*1000)), im)
                print "{}cross".format(label_color)
            elif abs(len(approx)-8) <= 2:  # circle

            	approx_contours.append(approx)
                circ_msg = String()
                circ_msg.data = "{} circle".format(label_color)
                self.pub_blobs.publish(circ_msg)
                moments = cv2.moments(c)
                center = (int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00']))
                im = passed_im.copy()
                cv2.putText(im, "{} circle".format(label_color), center, cv2.FONT_HERSHEY_PLAIN, 2, (100, 255, 100))
                cv2.drawContours(im, [approx], -1, (100, 255, 100), 2)
                cv2.imwrite("/home/ubuntu/challenge_photos1/{}circle{}.png".format(label_color, int(time.clock()*1000)), im)
                print "{}circle".format(label_color)
            if self.isTesting:
                cv2.drawContours(self.image, approx_contours, -1, (100, 255, 100), 2)
            #else:
            #    cv2.drawContours(im, approx_contours, -1, (100, 255, 100), 2)


if __name__ == "__main__":
    rospy.init_node("BlobDetector")
    bd = BlobDetector()
    rospy.spin()
    cv2.destroyAllWindows()
