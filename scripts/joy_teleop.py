#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy


MAX_SPEED = 2

class JoyController:
    def __init__(self):
        self.vesc_pub = rospy.Publisher("vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=0)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.cmd_cb)

    def cmd_cb(self, msg):
        # axes[0] x axis of left stick
        # axes[1] y axis of left stick
        # axes[2] left trigger
        # axes[3] x axis of right stick
        # axes[4] y axis of right stick
        # axes[5] right trigger
        # axes[6] x axis of directional buttons
        # axes[7] y axis of directional buttons
        # buttons[0] A
        # buttons[1] B
        # buttons[2] C
        # buttons[3] D
        # buttons[4] left bumper
        # buttons[5] right bumper
        # buttons[6] back
        # buttons[7] start
        # buttons[8] Logitech button
        # buttons[9] left stick
        # buttons[10] right stick
        cmd = AckermannDriveStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.drive.steering_angle = msg.axes[0]
        cmd.drive.speed = msg.axes[4] * MAX_SPEED
        self.vesc_pub.publish(cmd)


if __name__ == "__main__":
    rospy.init_node("joy_controller")
    jc = JoyController()
    rospy.spin()
