#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool


class NavigationMapServer:
    """
    This node publishes altered maps used for navigation.
    """

    def __init__(self):
        self.map_pub = rospy.Publisher("navigation_map", OccupancyGrid, queue_size=0)
        self.map_sub = rospy.Subscriber("map", OccupancyGrid, self.map_cb)
        self.lost_sub = rospy.Subscriber("is_lost", Bool, self.lost_cb)

        self.is_lost = False

    def map_cb(self, msg):
        altered_map = OccupancyGrid()

        # TODO: Stitch maps if lost
        if self.is_lost:
            pass

        # Set all unknown space as open
        altered_map.data = [cell if cell != -1 else 0 for cell in msg.data]
        
        self.map_pub.publish(altered_map)

    def lost_cb(self, msg):
        self.is_lost = msg.data


if __name__ == "__main__":
    rospy.init_node("navigation_map_server")
    server = NavigationMapServer()
    rospy.spin()
