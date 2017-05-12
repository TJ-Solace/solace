#!/usr/bin/env python

import rospy
from math import exp
from solace.msg import DriveCommand
from vesc_msgs.msg import VescStateStamped
from std_msgs.msg import Float64, Header


class ExpSmoother():
    def __init__(self, timeConst, debug=False):
        self.debug = debug
        self.timeConst = timeConst
        self.lastSample = 0.0
        self.lastRet = 0.0
        self.lastTime = None

    def sample(self, sample, time):
        if self.lastTime is None:
            self.lastTime = time
            return 0.0
        dt = (time - self.lastTime).to_sec()
        if dt == 0:  # if no time elapsed, just send the same thing as last time
            return self.lastSample
        a = dt / self.timeConst
        u = exp(-1.0 * a)
        v = (1.0 - u) / a
        ret = (u * self.lastRet) + ((v - u) * self.lastSample) + ((1.0 - v) * sample)
        if self.debug:
            rospy.loginfo_throttle(0.5, "input, output: " + repr(sample) + ", " + repr(ret))  # just to see
        self.lastRet = ret
        self.lastSample = sample
        self.lastTime = time
        return ret


class PhysicalControl():
    steering_mid = 0.463
    steering_mult = 0.45
    power_mult = 20000.0

    magic_current_number = 0.66  # numbers close to 1 aggressively reduce power on overcurrent, numbers close to 0 make less impact
    max_current = 45.0  # soft max current- set vesc maxima to higher than actually desired and let the node try to help first
    min_voltage = 3.2 * 4  # min 3.2 volts per cell

    # time constant for the exponential weighting in seconds- 68% of the output comes from 1tc in the past or less
    power_input_smoothing_tc = 0.13  # just trying to stop really aggressive reversal in direction

    current_input_smoothing_tc = 0.02  # only trying to smooth out the very sharp current spikes
    voltage_input_smoothing_tc = 1.0  # not looking for sudden drops, looking for longish-term trend of the battery being low

    def __init__(self):
        self.brake_pub = rospy.Publisher("/vesc/commands/motor/brake", Float64, queue_size=0)
        self.power_pub = rospy.Publisher("/vesc/commands/motor/speed", Float64, queue_size=0)
        self.steering_pub = rospy.Publisher("/vesc/commands/servo/position", Float64, queue_size=0)
        self.scan_sub = rospy.Subscriber("/drive", DriveCommand, self.command)
        self.vesc_sub = rospy.Subscriber("/vesc/sensors/core", VescStateStamped, self.drive)
        self.desired_speed = Float64()
        self.desired_angle = Float64()
        self.current_smoother = ExpSmoother(self.current_input_smoothing_tc)  # try to ignore the reasonably short current transients caused by accelerating sharply
        self.voltage_smoother = ExpSmoother(self.voltage_input_smoothing_tc)  # ignore heavy voltage drop transients from the same
        self.voltage_smoother.lastSample = 3.8 * 4  # cheat so it doesn't take a long time to become drivable
        self.voltage_smoother.lastRet = 3.8 * 4
        self.power_smoother = ExpSmoother(self.power_input_smoothing_tc)  # make the stick inputs chill a little bit

    def command(self, msg):
        thisT = self.get_time(msg.header.stamp)
        self.desired_angle = msg.steering * self.steering_mult + self.steering_mid
        self.desired_speed = self.power_smoother.sample(msg.power * self.power_mult, thisT)
        # self.desired_speed = msg.power * self.power_mult

    def drive(self, msg):
        thisT = self.get_time(msg.header.stamp)
        #rospy.loginfo_throttle(0.5, "actual speed: " + repr(msg.state.speed))  # just to see
        current = self.current_smoother.sample(msg.state.current_motor, thisT)
        voltage = self.voltage_smoother.sample(msg.state.voltage_input, thisT)
        if voltage < self.min_voltage:  # don't get to do anything if the battery is low
            rospy.logfatal_throttle(3.0, "Battery is too low! (" + repr(voltage) + "v)")
            self.power_pub.publish(0)
            self.steering_pub.publish(self.steering_mid)
            return

        self.steering_pub.publish(self.desired_angle)

        if current > self.max_current:
            rospy.logwarn_throttle(0.5, "current limiting: " + repr(current))
            self.desired_speed = self.power_smoother.sample((msg.state.speed * self.magic_current_number + self.desired_speed * (1 - self.magic_current_number)), thisT)
        if abs(msg.state.speed - self.desired_speed) / self.power_mult > 0.15 and ((msg.state.speed > 0 and self.desired_speed < msg.state.speed) or (msg.state.speed < 0 and self.desired_speed > msg.state.speed)):  # if we're reducing speed rapidly, brake
            rospy.loginfo_throttle(0.5, "braking")
            self.brake_pub.publish(self.max_current * 0.75)
            return
        self.power_pub.publish(self.desired_speed)

    @staticmethod
    def get_time(stamp):
        if stamp is None or stamp.to_sec() < 0.1:
            rospy.logerr("received bad header >.>")
            return rospy.Time.now()
        return stamp


if __name__ == '__main__':
    rospy.init_node("physical control")
    pc = PhysicalControl()
    rospy.spin()
