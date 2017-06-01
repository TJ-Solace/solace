#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from solace.msg import DriveCommand


class JoyController:
    def __init__(self):
        self.drive_pub = rospy.Publisher("/drive", DriveCommand, queue_size=0)
        self.joy_sub = rospy.Subscriber("/vesc/joy", Joy, self.cmd_cb)

        self.drive_msg = DriveCommand()

        self.enabled = True
        print "Initialized teleop"

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
        # buttons[2] X
        # buttons[3] Y
        # buttons[4] left bumper
        # buttons[5] right bumper
        # buttons[6] back
        # buttons[7] start
        # buttons[8] Logitech button
        # buttons[9] left stick
        # buttons[10] right stick

        if msg.buttons[5]:
            self.enabled = True
        elif msg.buttons[4]:
            self.enabled = False
	    self.drive_msg.power = 0
	    self.drive_msg.steering = 0
	    self.drive_msg.header.stamp = rospy.Time.now()
	    self.drive_pub.publish(self.drive_msg)

        if not self.enabled:
            return

        if -(msg.axes[5] - 1.0) > 0.01:  # braking
            self.drive_msg.power = 0
            self.drive_msg.steering = 0
            self.drive_msg.header.stamp = rospy.Time.now()
            self.drive_pub.publish(self.drive_msg)
        else:
            self.drive_msg.power = msg.axes[1] 
            self.drive_msg.steering = msg.axes[3]
            self.drive_msg.header.stamp = rospy.Time.now()
            self.drive_pub.publish(self.drive_msg)


if __name__ == "__main__":
    rospy.init_node("joy_controller")
    jc = JoyController()
    rospy.spin()
