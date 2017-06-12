#!/usr/bin/env python

import numpy as np
import subprocess
import os

import cv2
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from std_msgs.msg import Bool

SOLACE_PATH = "/home/ubuntu/racecar-ws/src/racecar/solace/"
STITCHING_PATH = "{}OpenPano/src/image-stitching".format(SOLACE_PATH)
MAP_DIR_PATH = "/home/ubuntu/.ros/"
GMAPPING_MAP_PATH = "{}gmapping_map.pgm".format(MAP_DIR_PATH)
CLEARED_GMAPPING_MAP_PATH = "{}cleared_gmapping_map.pgm".format(MAP_DIR_PATH)
GMAPPING_MAP_PATH_JPG = "{}gmapping_map.jpg".format(MAP_DIR_PATH)
FULL_MAP_PATH = "{}full_map.pgm".format(MAP_DIR_PATH)
FULL_MAP_PATH_JPG = "{}full_map.jpg".format(MAP_DIR_PATH)

OCC_THRESH = 0.65
FREE_THRESH = 0.196


class NavigationMapServer:
    """
    This node publishes altered maps used for navigation.
    """

    def __init__(self):
        self.map_pub = rospy.Publisher("navigation_map", OccupancyGrid, queue_size=0, latch=True)
        self.gmapping_disk_map_pub = rospy.Publisher("gmapping_disk_map", OccupancyGrid, queue_size=0)
        self.gmapping_map_sub = rospy.Subscriber("gmapping_map", OccupancyGrid, self.gmapping_map_cb)
        self.lost_sub = rospy.Subscriber("is_lost", Bool, self.lost_cb)

        self.is_lost = False
        self.map_msg = OccupancyGrid()

        rospy.wait_for_service("static_map")
        init_map_service = rospy.ServiceProxy("static_map", GetMap)
        self.pub_cleared_map(init_map_service().map)
        rospy.loginfo("published initial map")

    def gmapping_map_cb(self, msg):
        if self.is_lost:
            #self.gmapping_disk_map_pub.publish(msg)  # save gmapping map to disk
            # stitch gmapping map to full map
            try:
                # clear gmapping map
                #gmapping_map = cv2.imread(GMAPPING_MAP_PATH)
		#rospy.logwarn(os.path.exists(GMAPPING_MAP_PATH))
		#rospy.loginfo(gmapping_map)
                #obstacles = cv2.inRange(gmapping_map, np.array([1, 1, 1]), np.array([255, 255, 255]))
                #cv2.imwrite(CLEARED_GMAPPING_MAP_PATH, obstacles)
		msg.data = self.clear_map(msg.data)
                self.gmapping_disk_map_pub.publish(msg)  # save gmapping map to disk
                rospy.loginfo("cleared gmapping map")

                # convert pgms to jpgs
                subprocess.call(["convert", FULL_MAP_PATH, FULL_MAP_PATH_JPG], stderr=subprocess.STDOUT)
                subprocess.call(["convert", CLEARED_GMAPPING_MAP_PATH, GMAPPING_MAP_PATH_JPG], stderr=subprocess.STDOUT)

                # smooth the gmapping map
                img = cv2.imread(GMAPPING_MAP_PATH_JPG)
                blurred = cv2.GaussianBlur(img, (21, 21), 0)
                smoothed = cv2.Canny(blurred, 10, 115, apertureSize=3)
                smoothed = cv2.bitwise_not(smoothed)
                cv2.imwrite(GMAPPING_MAP_PATH_JPG, smoothed)
                rospy.loginfo("smoothed the gmapping map")

                # stitch
                #rospy.loginfo("{} {} {}".format(STITCHING_PATH, FULL_MAP_PATH_JPG, GMAPPING_MAP_PATH_JPG))
                subprocess.check_call(
                    ["bash", "-c", "{} {} {}".format(STITCHING_PATH, FULL_MAP_PATH_JPG, GMAPPING_MAP_PATH_JPG)],
                    stderr=subprocess.STDOUT)
                rospy.loginfo("successfully stitched new map!")

                # convert to pgm and publish
                subprocess.call(["convert", "-compress", "none", "out.jpg", FULL_MAP_PATH], stderr=subprocess.STDOUT)
                self.map_msg.header.stamp = rospy.Time.now()
                self.map_msg.info.map_load_time = rospy.Time.now()
                self.file_to_occupancygrid(FULL_MAP_PATH, self.map_msg)
                self.map_pub.publish(self.map_msg)
                rospy.loginfo("successfully loaded new map!")
            except subprocess.CalledProcessError:
                rospy.logerr("failed to stitch!")
        else:
            rospy.loginfo("Got a gmapping map, but I'm not lost.")

    def lost_cb(self, msg):
        self.is_lost = msg.data

    def clear_map(self, map_arr):
        return [occ if occ >= OCC_THRESH else 0 for occ in map_arr]

    def pub_cleared_map(self, msg):
        """
        This function publishes the map with all unknown areas marked as open. 
        """
        # Set all unknown space as open
        msg.data = [cell if cell != -1 else 0 for cell in msg.data]
        self.map_pub.publish(msg)
        rospy.loginfo("published navigation map")

    @staticmethod
    def file_to_occupancygrid(file_path, grid_msg):
        with open(file_path, "r") as infile:
            inpt = infile.read().split()
        grid_msg.info.width, grid_msg.info.height = int(inpt[1]), int(inpt[2])
        max_intensity = float(inpt[3])
        # convert to ternary occupancy values: 0 if free, 100 if occupied, -1 if unknown
        grid_msg.data = [0 if (max_intensity - int(p)) / max_intensity <= FREE_THRESH else -1 if (max_intensity - int(
            p)) / max_intensity < OCC_THRESH else 100 for p in inpt[4:]]


if __name__ == "__main__":
    rospy.init_node("navigation_map_server")
    server = NavigationMapServer()
    rospy.spin()
