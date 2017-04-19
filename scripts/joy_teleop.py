#!/usr/bin/env python

import rospy
import math
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

mults = [0.25, 20, 20000]
offsets = [0, 0, 0]

servo_min = 0.1
servo_max = 0.9
servo_offset = 0.463


class JoyController:
    def __init__(self):
        self.sevo_pub = rospy.Publisher("/vesc/commands/servo/position", Float64, queue_size=0)
        self.duty_pub = rospy.Publisher("/vesc/commands/motor/duty_cycle", Float64, queue_size=0)
        self.brake_pub = rospy.Publisher("/vesc/commands/motor/brake", Float64, queue_size=0)
        self.current_pub = rospy.Publisher("/vesc/commands/motor/current", Float64, queue_size=0)
        self.speed_pub = rospy.Publisher("/vesc/commands/motor/speed", Float64, queue_size=0)
        self.joy_sub = rospy.Subscriber("/vesc/joy", Joy, self.cmd_cb)
        self.prev_speed = 0
        self.prev_time = rospy.get_time()
        self.killed = False
        self.mode = 0
        self.braking = False
        print "Initialized teleop"
        # 0 == duty cycle
        # 1 == current
        # 2 == speed

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

        if msg.buttons[4]:
            self.killed = False

        if msg.buttons[5]:
            cmd = Float64()
            cmd.data = 0
            self.current_pub.publish(cmd)
            self.killed = True

        if self.killed:
            return

        if msg.buttons[0]:
            self.mode = 0
            print "duty cycle mode"
        elif msg.buttons[1]:
            self.mode = 1
            print "current mode"
        elif msg.buttons[2]:
            self.mode = 2
            print "speed mode"

        if -(msg.axes[5] - 1) > 0.01:
            self.braking = True
            brake = Float64()
            brake.data = -(msg.axes[5] - 1) * 25
            self.brake_pub.publish(brake)
            self.setServoPos(msg.axes[3] / -2 + servo_offset)
            return

        if self.braking:
            self.braking = False
            brake = Float64()
            brake.data = 0
            self.brake_pub.publish(brake)

        if self.mode == 0:
            self.setDuty(msg.axes[1] * mults[0] + offsets[0])
            self.setServoPos(msg.axes[3] / -2 + servo_offset)
        elif self.mode == 1:
            self.setCurrent(msg.axes[1] * mults[1] + offsets[1])
            self.setServoPos(msg.axes[3] / -2 + servo_offset)
        elif self.mode == 2:
            self.setSpeed(msg.axes[1] * mults[2] + offsets[2])
            self.setServoPos(msg.axes[3] / -2 + servo_offset)

    def setDuty(self, duty):
        cmd = Float64()
        print "Duty command is " + str(duty)
        self.duty_pub.publish(cmd)

    def setCurrent(self, current):
        cmd = Float64()
        cmd.data = 0
        self.current_pub.publish(cmd)

    def setSpeed(self, speed):
        cmd = Float64()
        cmd.data = speed
        self.speed_pub.publish(speed)

    def setServoPos(self, pos):
        cmd = Float64()
        cmd.data = servo_min
        if pos > servo_max:
            cmd.data = servo_max
        elif pos > servo_min:
            cmd.data = pos
        self.sevo_pub.publish(cmd)


if __name__ == "__main__":
    rospy.init_node("joy_controller")
    jc = JoyController()
    rospy.spin()
