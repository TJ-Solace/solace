#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu


ORIENTATION_COV = [1.73e-6, 0.069, 0.071,
                   0.069, 6.92e-7, 0.044,
                   0.071, 0.044, 6.67e-6]
ANGULAR_VEL_COV = [7.37e-6, -0.03, 0.072,
                   -0.03, 2.88e-6, -0.01,
                   0.07, -0.01, 3.68e-6]
LINEAR_ACC_COV = [1.62e-3, 0.0, 0.0,
                  0.0, 5.67e-3, 0.0,
                  0.0, 0.0, 2.30e-3]


class ImuServer:
    """
    This node publishes altered IMU messages.
    """

    def __init__(self):
        self.imu_pub = rospy.Publisher("/imu_fixed", Imu, queue_size=0)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_cb)

    def imu_cb(self, msg):
        """
        This function publishes an altered IMU message with the covariance matrices set. 
        """
        msg.orientation_covariance = ORIENTATION_COV
        msg.angular_velocity_covariance = ANGULAR_VEL_COV
        msg.linear_acceleration_covariance = LINEAR_ACC_COV

        self.imu_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("imu_server")
    server = ImuServer()
    rospy.spin()
