#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy

class JoyController:
    def __init__(self):
        self.vesc_pub = rospy.Publisher("vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=0)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.cmd_cb)

    def cmd_cb(self, msg):
        # Axes[0] x axis of left stick
        # Axes[1] y axis of left stick
        # Axes[2] left trigger
        # Axes[3] x axis of right stick
        # Axes[4] y axis of right stick
        # Axes[5] right trigger
        # Axes[6] x axis of directional buttons
        # Axes[7] y axis of directional buttons
        # Buttons[0] A
        # Buttons[1] B
        # Buttons[2] C
        # Buttons[3] D
        # Buttons[4] left bumper
        # Buttons[5] right bumper
        # Buttons[6] back
        # Buttons[7] start
        # Buttons[8] Logitech button
        # Buttons[9] left stick
        # Buttons[10] right stick

