#!/usr/bin/env python
import time

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

ODOM_TOPIC = "/ucl_0/vrpn_client/estimated_odometry"
VICON_TOPIC = "/vrpn_client_node/ucl_0/pose"

timeout_threshold = 1.0  # in seconds
odom_last_update = None
vicon_last_update = None


def odom_cb(msg):
    global odom_last_update
    odom_last_update = time.time()


def vicon_cb(msg):
    global vicon_last_update
    vicon_last_update = time.time()


if __name__ == "__main__":
    rospy.init_node('trajectory_circle', anonymous=True)
    node_rate = rospy.Rate(1)
    rospy.Subscriber(ODOM_TOPIC, Odometry, odom_cb)
    rospy.Subscriber(VICON_TOPIC, PoseWithCovarianceStamped, vicon_cb)

    while not rospy.is_shutdown():
        # Check odom
        if odom_last_update and (time.time() - odom_last_update) > timeout_threshold:
            rospy.logerr("Odom is NOT PUBLISHING!")
            exit(-1)
        else:
            rospy.loginfo("Odom is ok!")

        # Check vicon
        if vicon_last_update and (time.time() - vicon_last_update) > timeout_threshold:
            rospy.logerr("VICON is NOT PUBLISHING!")
            exit(-1)
        else:
            rospy.loginfo("VICON is ok!")

        node_rate.sleep()
