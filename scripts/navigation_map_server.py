#!/usr/bin/env python

import subprocess

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool

SOLACE_PATH = "/home/ubuntu/racecar-ws/src/racecar/solace/"
STITCHING_PATH = "{}OpenPano/src/image-stitching".format(SOLACE_PATH)
MAP_DIR_PATH = "{}maps/".format(SOLACE_PATH)
GMAPPING_MAP_PATH = "{}gmapping_map.pgm".format(MAP_DIR_PATH)
FULL_MAP_PATH = "{}full_map.pgm".format(MAP_DIR_PATH)

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
        self.init_map_sub = rospy.Subscriber("map", OccupancyGrid, self.open_map_cb)
        self.lost_sub = rospy.Subscriber("is_lost", Bool, self.lost_cb)

        self.is_lost = False
        self.map_msg = OccupancyGrid()

    def open_map_cb(self, msg):
        """
        This function publishes the map with all unknown areas marked as open. 
        """
        self.map_msg = msg
        actual_map = msg.data

        # Set all unknown space as open
        self.map_msg.data = [cell if cell != -1 else 0 for cell in self.map_msg.data]
        self.map_pub.publish(self.map_msg)

        # Revert to actual map
        self.map_msg.data = actual_map

    def gmapping_map_cb(self, msg):
        if self.is_lost:
            self.gmapping_disk_map_pub.publish(msg)  # save gmapping map to disk
            # stitch gmapping map to full map
            try:
	    	rospy.logwarn("{} {} {}".format(STITCHING_PATH, FULL_MAP_PATH, GMAPPING_MAP_PATH))
                # TODO: remove shell=True if possible
                subprocess.check_call([STITCHING_PATH, FULL_MAP_PATH, GMAPPING_MAP_PATH], stderr=subprocess.STDOUT, shell=True)
                subprocess.call(["convert", "-compress", "none", "out.jpg", FULL_MAP_PATH], stderr=subprocess.STDOUT, shell=True)
                self.map_msg.header.stamp = rospy.Time.now()
                self.map_msg.info.map_load_time = rospy.Time.now()
                self.file_to_occupancygrid(FULL_MAP_PATH, self.map_msg)
                rospy.loginfo("successfully stitched new map!")
            except subprocess.CalledProcessError:
                rospy.logerr("failed to stitch!")
            self.open_map_cb(self.map_msg)
        else:
            rospy.loginfo("Got a gmapping map, but I'm not lost.")

    def lost_cb(self, msg):
        self.is_lost = msg.data

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
