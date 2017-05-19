#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu


ORIENTATION_COV = [1.0, 0.0, 0.0,
                   0.0, 1.0, 0.0,
                   0.0, 0.0, 1.0]
ANGULAR_VEL_COV = [1.0, 0.0, 0.0,
                   0.0, 1.0, 0.0,
                   0.0, 0.0, 1.0]
LINEAR_ACC_COV = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]


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
