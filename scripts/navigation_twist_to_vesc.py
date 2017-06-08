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
        self.cmd_sub = rospy.Subscriber("/cmd_vel", Twist, self.vel_cb)

        self.drive_msg = DriveCommand()

    def vel_cb(self, msg):
        rospy.loginfo("{} {}".format(msg.linear.x, msg.linear.y))
        self.drive_msg.power = math.sqrt(msg.linear.x ** 2 + msg.linear.y ** 2) * 10
        # TODO: check if steering angle is calculated correctly (negate? switch y and x?)
        self.drive_msg.steering = math.atan2(msg.linear.y, msg.linear.x) / math.pi
        if self.drive_msg.power < 0:  # negate angle if driving backwards
            self.drive_msg.steering *= -1

        self.drive_msg.header.stamp = rospy.Time.now()
        self.drive_pub.publish(self.drive_msg)

        rospy.loginfo("published a drive message from navigation stack. power {}, angle {}".format(self.drive_msg.power, self.drive_msg.steering))


if __name__ == "__main__":
    rospy.init_node("navigation_twist_to_vesc")
    nttv = NavigationTwistToVESC()
    rospy.spin()
