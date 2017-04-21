#!/usr/bin/env python

import math

import rospy
from geometry_msgs.msg import Twist
from solace.msg import DriveCommand


class NavigationTwistToVESC:
    """
    This node listens to the cmd_vel topic published by the navigation stack and converts messages to commands sent to the VESC.
    """

    def __init__(self):
        self.drive_pub = rospy.Publisher("/drive", DriveCommand, queue_size=0)
        self.cmd_sub = rospy.Subscriber("/vel_cb", Twist, self.vel_cb)

        self.drive_msg = DriveCommand()

    def vel_cb(self, msg):
        self.drive_msg.power = math.sqrt(msg.linear.x ** 2 + msg.linear.y ** 2)
        self.drive_msg.steering = msg.angular.z

        self.drive_pub.publish(self.drive_msg)


if __name__ == "__main__":
    rospy.init_node("navigation_twist_to_vesc")
    nttv = NavigationTwistToVESC()
    rospy.spin()
