#!/usr/bin/env python

import numpy as np
import cv2


the_map = cv2.imread("unsmoothed.pgm")
obstacles = cv2.inRange(the_map, np.array([1, 1, 1]), np.array([255, 255, 255]))
cv2.imwrite("cleared.jpg", obstacles)
