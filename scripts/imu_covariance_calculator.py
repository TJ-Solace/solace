import numpy as np
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from tf import transformations
import rospy


class CommonCoincidentalCounterproductiveCovarianceCalibrationConvenienceCalculatorClass():
    dt = 0.05
    num_samples = 1000
    save_path = "/home/ubuntu/covar.txt"

    def __init__(self):
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.gather)
        self.orientation_samples = np.zeros((3, self.num_samples))
        self.anglular_velocity_samples = np.zeros((3, self.num_samples))
        self.linear_accel_samples = np.zeros((3, self.num_samples))
        self.step = 0
	rospy.loginfo("initialized covariance calculator")

    def gather(self, msg):
        rospy.loginfo(str(self.orientation_samples))
    	rospy.loginfo_throttle(0.5, self.step)
        roll, pitch, yaw = transformations.euler_from_quaternion((msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))
        roll_velocity, pitch_velocity, yaw_velocity = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        x_accel, y_accel, z_accel = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        self.orientation_samples[:, self.step] = [roll, pitch, yaw]
        self.anglular_velocity_samples[:, self.step] = [roll_velocity, pitch_velocity, yaw_velocity]
        self.linear_accel_samples[:, self.step] = [x_accel, y_accel, z_accel]
        self.step += 1
        if self.step >= self.num_samples:
            # we're done
            rospy.loginfo("filled sample buffer")
            orientation_covar = np.cov(self.orientation_samples)
            ang_vel_covar = np.cov(self.anglular_velocity_samples)
            lin_accel_covar = np.cov(self.linear_accel_samples)
            rospy.loginfo("saving values")
            with open(self.save_path, "w") as f:
                f.write("orientation:\n")
                f.write(str(orientation_covar) + '\n')
                f.write("angular velocity:\n")
                f.write(str(ang_vel_covar) + '\n')
                f.write("linear acceleration:\n")
                f.write(str(lin_accel_covar) + '\n')
            rospy.loginfo("finished saving")
            rospy.signal_shutdown("common coincidental calibrated covariances counterproductively calculated conveniently")

if __name__ == '__main__':
    rospy.init_node("imu_covariance_calculator")
    CCCCCCCC = CommonCoincidentalCounterproductiveCovarianceCalibrationConvenienceCalculatorClass()
    rospy.spin()
