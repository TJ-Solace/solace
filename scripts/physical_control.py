#!/usr/bin/env python

import rospy
from math import exp
from solace.msg import DriveCommand
from std_msgs.msg import Float64


class ExpSmoother():
    def __init__(self, alpha):
        self.alpha = alpha
        self.lastSample = 0
        self.lastRet = 0

    def sample(self, sample, dt):
        a = dt / self.alpha
        u = exp(-1 * a)
        v = (1 - u) / a
        ret = (u * self.lastRet) + ((v - u) * self.lastSample) + ((1.0 - v) * sample)
        self.lastRet = ret
        self.lastSample = sample
        return ret


class PhysicalControl():
    steering_mid = 0.463
    steering_mult = 0.45
    power_mult = 20000

    power_input_smoothing_rate = 0.99  # currently blind guess, probably empirically determined eventually

    current_input_smoothing_rate = 0.99  # currently blind guess, ditto

    max_current = 60.0

    def __init__(self):
        self.power_pub = rospy.Publisher("/vesc/commands/motor/speed", Float64, queue_size=0)
        self.steering_pub = rospy.Publisher("/vesc/commands/servo/position", Float64, queue_size=0)
        self.scan_sub = rospy.Subscriber("/drive", DriveCommand, self.command)
        self.vesc_sub = rospy.Subscriber("/vesc/state", VescStateStamped, self.drive)  # TODO: actually figure out what this is
        self.desired_speed = 0
        self.desired_angle = 0

    def command(self, msg):
        self.desired_angle = msg.steering * self.steering_mult + self.steering_mid
        self.desired_speed = msg.power * self.power_mult

    def drive(self, msg):
        if abs(msg.current_motor) > self.max_current:
            self.desired_speed /= 2
        self.power_pub.publish(self.desired_speed)
        self.steering_pub.publish(self.desired_angle)


if __name__ == '__main__':
    rospy.init_node("physical control")
    pc = PhysicalControl()
    rospy.spin()
