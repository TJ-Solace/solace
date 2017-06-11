#!/usr/bin/env python

import cv2


img = cv2.imread("unsmoothed.pgm")
blurred = cv2.GaussianBlur(img, (21, 21), 0)
smoothed = cv2.Canny(blurred, 10, 115, apertureSize=3)
smoothed = cv2.bitwise_not(smoothed)
cv2.imwrite("smoothed.jpg", smoothed)
