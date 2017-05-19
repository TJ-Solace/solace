#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool


class LostDetector:
    def __init__(self):
        self.lost_pub = rospy.Publisher("is_lost", Bool, queue_size=0, latch=True)
        self.poses_sub = rospy.Subscriber("particlecloud", PoseArray, self.poses_cb)
        self.is_lost = False
        self.msg = Bool()
        self.lost_start = rospy.get_time()

    def poses_cb(self, msg):
    	rospy.loginfo("number of particles: {}".format(len(msg.poses)))
        if len(msg.poses) > 1500:  # lost
            if not self.is_lost:
                self.is_lost = True
                self.lost_start = rospy.get_time()
            elif rospy.get_time() - self.lost_start > 3.0:  # Publish that the car is lost if it's been lost for 3 seconds
                self.msg.data = True
                rospy.loginfo("lost!")
                self.lost_pub.publish(self.msg)
        elif self.is_lost:
            self.is_lost = False
            self.msg.data = False
            rospy.loginfo("no longer lost!")
            self.lost_pub.publish(self.msg)


if __name__ == "__main__":
    rospy.init_node("lost_detector")
    ld = LostDetector()
    rospy.spin()
