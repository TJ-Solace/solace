#!/usr/bin/env python

import rospy
from solace.msg import DriveCommand
from std_msgs.msg import Bool, Float64


class PhysicalControl():
    steering_mid = 0.463
    steering_mult = 0.45
    power_mult = 20000

    def __init__(self):
        self.power_pub = rospy.Publisher("/vesc/commands/motor/speed", Float64, queue_size=0)
        self.steering_pub = rospy.Publisher("/vesc/commands/servo/position", Float64, queue_size=0)
        self.scan_sub = rospy.Subscriber("/drive", DriveCommand, self.drive)

    def drive(self, msg):
        servo_angle = msg.steering * self.steering_mult + self.steering_mid
        power = msg.power * self.power_mult
        self.power_pub.publish(power)
        self.steering_pub.publish(servo_angle)
