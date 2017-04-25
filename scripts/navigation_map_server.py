#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool



class NavigationMapServer:
    """
    This node publishes altered maps used for navigation.
    """

    def __init__(self):
        self.map_pub = rospy.Publisher("navigation_map", OccupancyGrid, queue_size=0, latch=True)
        self.disk_map_pub = rospy.Publisher("disk_map", OccupancyGrid, queue_size=0)
        self.map_sub = rospy.Subscriber("map", OccupancyGrid, self.map_cb)
        self.lost_sub = rospy.Subscriber("is_lost", Bool, self.lost_cb)

        self.is_lost = False
        self.map_msg = OccupancyGrid()
        self.lost_count = 0

    def map_cb(self, msg):
        # TODO: Stitch maps if lost and update map_msg metadata
        if self.is_lost:
            self.lost_count += 1
            if self.lost_count % 5 == 0:  # update map on disk every 5 updates
                self.disk_map_pub.publish(self.map_msg)
        else:
            self.map_msg = msg
        actual_map = self.map_msg.data

        # Set all unknown space as open
        self.map_msg.data = [cell if cell != -1 else 0 for cell in self.map_msg.data]
        self.map_pub.publish(self.map_msg)

        # Revert to actual map
        self.map_msg.data = actual_map

    def lost_cb(self, msg):
        self.is_lost = msg.data


if __name__ == "__main__":
    rospy.init_node("navigation_map_server")
    server = NavigationMapServer()
    rospy.spin()
