#!/usr/bin/env python

import rospy
import math
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

MAX_SPEED = 4
MAX_ACCELERATION = 1

class JoyController:
    def __init__(self):
        self.vesc_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/teleop", AckermannDriveStamped, queue_size=0)
        self.brake_pub = rospy.Publisher("/vesc/commands/motor/brake", Float64, queue_size=0)
        self.joy_sub = rospy.Subscriber("/vesc/joy", Joy, self.cmd_cb)
	self.prev_speed = 0
        self.prev_time = rospy.get_time()

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
	if -(msg.axes[5]-1) > 0:
            brake = Float64()
            brake.data = -(msg.axes[5]-1) * 25
            self.brake_pub.publish(brake)
	else:
	    cmd = AckermannDriveStamped()
            cmd.header.stamp = rospy.Time.now()
            speed = msg.axes[1] * MAX_SPEED
            try:
                if abs((speed - self.prev_speed) / (rospy.get_time() - self.prev_time)) > MAX_ACCELERATION:
                    speed = self.prev_speed + ((speed - self.prev_speed) / abs(speed - self.prev_speed)) * (MAX_ACCELERATION * (rospy.get_time() - self.prev_time))
            except: pass
            cmd.drive.speed = speed
            self.prev_speed = speed
            self.prev_time = rospy.get_time()
            cmd.drive.steering_angle = msg.axes[3] * math.pi/6
            self.vesc_pub.publish(cmd)


if __name__ == "__main__":
    rospy.init_node("joy_controller")
    jc = JoyController()
    rospy.spin()
