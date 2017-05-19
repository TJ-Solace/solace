import numpy as np
from sensor_msgs.msgs import Imu
import rospy


class CommonCoincidentalCounterproductiveCovarianceCalibrationCalculatorClass():
    dt = 0.05
    num_samples = 10000

    def __init__(self):
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.gather)

