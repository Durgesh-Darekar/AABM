#!/usr/bin/env python
from time import time

import rospy
from nav_msgs.msg import Odometry

ODOM_TOPIC = "/ucl_0/vrpn_client/estimated_odometry"
VICON_TOPIC = "/vrpn_client_node/ucl_0/pose"

timeout_threshold = 1.0  # in seconds
odom_last_update = None
vicon_last_update = None


def odom_cb(msg):
    if odom_last_update is None:
        odom_last_update = time()


def vicon_cb(msg):
    if vicon_last_update is None:
        vicon_last_update = time()



if __name__ == "__main__":
    rospy.init_node('trajectory_circle', anonymous=True)
    node_rate = rospy.Rate(1)
    rospy.Subscriber(ODOM_TOPIC, Odometry, odom_cb)
    rospy.Subscriber(VICON_TOPIC, PoseWithCovarianceStamped, vicon_cb)

    while True:
        # Check odom
        if (time() - odom_last_update) > timeout_threshold:
            rospy.logerr("Odom is NOT PUBLISHING!")
        else:
            rospy.loginfo("Odom is ok!")

        # Check vicon
        if (time() - vicon_last_update) > timeout_threshold:
            rospy.logerr("VICON is NOT PUBLISHING!")
        else:
            rospy.loginfo("VICON is ok!")

        node_rate.sleep()
