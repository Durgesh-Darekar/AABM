#!/usr/bin/env python
from time import time

import rospy
from nav_msgs.msg import Odometry

ODOM_TOPIC = "/ucl_0/vrpn_client/estimated_odometry"
VICON_TOPIC = "/vrpn_client_node/ucl_0/pose"

timeout_threshold = 1.0  # in seconds
odom_ok = True
odom_last_update = None
vicon_ok = True
odom_last_update = None


def odom_cb(msg):
    if odom_last_update is None:
        odom_last_update = time()

    if (time() - odom_last_update) > timeout_threshold:
        odom_ok = False


def vicon_cb(msg):
    if vicon_last_update is None:
        vicon_last_update = time()

    if (time() - vicon_last_update) > timeout_threshold:
        vicon_ok = False


if __name__ == "__main__":
    rospy.init_node('trajectory_circle', anonymous=True)
    node_rate = rospy.Rate(1)
    rospy.Subscriber(ODOM_TOPIC, Odometry, odom_cb)
    rospy.Subscriber(VICON_TOPIC, PoseWithCovarianceStamped, vicon_cb)

    while True:
        if odom_ok:
            rospy.loginfo("Odom is ok!")
        else:
            rospy.logerr("Odom is NOT PUBLISHING!")

        if vicon_ok:
            rospy.loginfo("VICON is ok!")
        else:
            rospy.logerr("VICON is NOT PUBLISHING!")

        node_rate.sleep()
