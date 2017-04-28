#!/usr/bin/env python

import rospy
from math import exp
from solace.msg import DriveCommand
from vesc_msgs.msg import VescStateStamped
from std_msgs.msg import Float64, Header


class ExpSmoother():
    def __init__(self, timeConst):
        self.timeConst = timeConst
        self.lastSample = 0
        self.lastRet = 0
        self.lastTime = None

    def sample(self, sample, time):
        if self.lastTime is None:
            self.lastTime = time
            return 0
        dt = (time - self.lastTime).to_sec()
        a = dt / self.timeConst
        u = exp(-1 * a)
        v = (1 - u) / a
        ret = (u * self.lastRet) + ((v - u) * self.lastSample) + ((1.0 - v) * sample)
        self.lastRet = ret
        self.lastSample = sample
        self.lastTime = time
        return ret


class PhysicalControl():
    steering_mid = 0.463
    steering_mult = 0.45
    power_mult = 20000

    max_current = 60.0
    min_voltage = 3.5 * 4  # min 3.5 volts per cell

    # time constant for the exponential weighting in seconds- 68% of the output comes from 1tc in the past or less
    power_input_smoothing_tc = 0.15  # just trying to stop really agressive reversal in direction

    current_input_smoothing_tc = 0.05  # only trying to smooth out the very sharp current spikes
    voltage_input_smoothing_tc = 3  # not looking for sudden drops, looking for longish-term trend of the battery being low

    def __init__(self):
        self.power_pub = rospy.Publisher("/vesc/commands/motor/speed", Float64, queue_size=0)
        self.steering_pub = rospy.Publisher("/vesc/commands/servo/position", Float64, queue_size=0)
        self.scan_sub = rospy.Subscriber("/drive", DriveCommand, self.command)
        self.vesc_sub = rospy.Subscriber("/vesc/sensors/core", VescStateStamped, self.drive)
        self.desired_speed = Float64()
        self.desired_angle = Float64()
        self.current_smoother = ExpSmoother(self.current_input_smoothing_tc)  # try to ignore the short and reasonably short current transients
        self.voltage_smoother = ExpSmoother(self.voltage_input_smoothing_tc)  # ignore heavy voltage drop transients from the same
        self.power_smoother = ExpSmoother(self.power_input_smoothing_tc)  # make the stick inputs chill a little bit

    def command(self, msg):
        thisT = msg.header.stamp
        self.desired_angle.data = msg.steering * self.steering_mult + self.steering_mid
        self.desired_speed.data = self.power_smoother.sample(msg.power * self.power_mult, thisT)

    def drive(self, msg):
        thisT = msg.header.stamp
        current = self.current_smoother.sample(msg.state.current_motor, thisT)
        voltage = self.voltage_smoother.sample(msg.state.voltage_input, thisT)
        if voltage < self.min_voltage:
            self.power_pub.publish(0)
            self.steering_pub.publish(self.steering_mid)
            return
        if current > self.max_current:
            self.desired_speed = self.power_smoother.sample((msg.state.speed + self.desired_speed) / 2.2, thisT)
        self.power_pub.publish(self.desired_speed)
        self.steering_pub.publish(self.desired_angle)


if __name__ == '__main__':
    rospy.init_node("physical control")
    pc = PhysicalControl()
    rospy.spin()
