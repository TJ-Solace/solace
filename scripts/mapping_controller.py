#!/usr/bin/env python

import roslaunch
import rospy
from std_msgs.msg import Bool

LAUNCH_FILE = "/home/ubuntu/racecar-ws/src/racecar/solace/launch/mapping.launch"


class MappingController:
    """
    This node starts the mapping stack when the car is lost, and stops it when the car is not.
    """

    def __init__(self):
        self.lost_sub = rospy.Subscriber("is_lost", Bool, self.lost_cb)

        self.launch = None

    def lost_cb(self, msg):
        if msg.data:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(uuid, [LAUNCH_FILE])
            self.launch.start()
            rospy.loginfo("started gmapping node")
        elif self.launch is not None:
            self.launch.shutdown()
            self.launch = None
            rospy.loginfo("stopped gmapping node")


if __name__ == "__main__":
    rospy.init_node("mapping_controller")
    mc = MappingController()
    rospy.spin()
