#!/usr/bin/env python
from math import cos
from math import sin
from math import radians

import numpy as np

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from aabm_comms.msg import Waypoint
from aabm_comms.msg import Trajectory
from aabm_comms.msg import Trajectory_sngl

NB_WAYPOINTS = 1200
CIRCLE_DIAMETER = 2.3
TRAJ_ALTITUDE = 0.5
TRAJ_REF_TOPIC = '/ucl_0/autopilot/TrajectoryReference'
ODOM_TOPIC = '/ucl_0/vrpn_client/estimated_odometry'


def generate_circle_trajectory(nb_waypoints, diameter):
    # Starting co-ordinates
    x_start = 0.0
    y_start = 0.0

    # Circle properties
    section_angle = 360.0 / nb_waypoints
    radius = diameter / 2.0

    # Generate waypoints
    waypoints = np.zeros((nb_waypoints, 2))
    for i in range(nb_waypoints):
        angle = radians(section_angle * i)

        if 0 <= angle and angle <= radians(90):
            cos_x = radius * cos(angle)
            sin_y = radius * sin(angle)
            x_pos = x_start + (radius - cos_x)
            y_pos = y_start + sin_y
            waypoints[i, 0] = x_pos
            waypoints[i, 1] = y_pos

        elif angle <= radians(180):
            ang = angle - radians(90)
            sin_x = radius * sin(ang)
            cos_y = radius * cos(ang)
            x_pos = x_start + (radius + sin_x)
            y_pos = y_start + cos_y
            waypoints[i, 0] = x_pos
            waypoints[i, 1] = y_pos

        elif angle <= radians(270):
            ang = angle - radians(180)
            cos_x = radius * cos(ang)
            sin_y = radius * sin(ang)
            x_pos = x_start + (radius + cos_x)
            y_pos = y_start - sin_y
            waypoints[i, 0] = x_pos
            waypoints[i, 1] = y_pos

        elif angle <= radians(360):
            ang = angle - radians(270)
            sin_x = radius * sin(ang)
            cos_y = radius * cos(ang)
            x_pos = x_start + (radius - sin_x)
            y_pos = y_start - cos_y
            waypoints[i, 0] = x_pos
            waypoints[i, 1] = y_pos

    return waypoints


def test_generate_circle_trajectory():
    import matplotlib.pylab as plt
    waypoints = generate_circle_trajectory(NB_WAYPOINTS, CIRCLE_DIAMETER)
    plt.plot(waypoints[:, 0], waypoints[:, 1])
    plt.show()


if __name__ == '__main__':
    # Start ROS node
    rospy.init_node('trajectory_circle', anonymous=True)
    node_rate = rospy.Rate(1)
    traj_pub = rospy.Publisher(TRAJ_REF_TOPIC, Trajectory, queue_size=10)
    clear_queue = True # Do not need to clear previous trajectory

    # Wait 5 secs to allow the ros node to initialise
    rospy.loginfo('Starting ros node ...')
    for i in range(5):
        rospy.loginfo(5 - i)
        node_rate.sleep()

    # Get drone odometery
    rospy.loginfo('Waiting for drone odom msg')
    drone_odom = rospy.wait_for_message(ODOM_TOPIC, Odometry)
    rospy.loginfo(str(drone_odom))

    # Build trajectory
    rospy.loginfo('Generating circle trajectory')
    circle_traj = Trajectory()
    # -- Setting initial waypoint which should be the current position.
    wp_init = Waypoint()
    wp_init.position.x = drone_odom.pose.pose.position.x
    wp_init.position.y = drone_odom.pose.pose.position.y
    wp_init.position.z = drone_odom.pose.pose.position.z
    circle_traj.initialWaypoint = wp_init
    # -- Start the trajectory with the 0th point (same as the initial waypoint).
    traj_single = Trajectory_sngl()
    traj_single.position.x = wp_init.position.x
    traj_single.position.y = wp_init.position.y
    traj_single.position.z = wp_init.position.z
    traj_single.timeMilliseconds = 0
    circle_traj.trajectory.append(traj_single)
    # -- Issuing a point for start point, to ensure it stays same always
    traj_single = Trajectory_sngl()
    traj_single.position.x = 0.0
    traj_single.position.y = 0.0
    traj_single.position.z = TRAJ_ALTITUDE
    traj_single.yaw = 0
    traj_single.timeMilliseconds = 3000  # Give MAV 3s to reach start position
    circle_traj.trajectory.append(traj_single)
    # -- Generate trajectory waypoints
    waypoints = generate_circle_trajectory(NB_WAYPOINTS, CIRCLE_DIAMETER)
    assert(waypoints.shape[0] == NB_WAYPOINTS)
    assert(waypoints.shape[1] == 2)
    for i in range(NB_WAYPOINTS):
        traj_single = Trajectory_sngl()
        traj_single.position.x = waypoints[i, 0]
        traj_single.position.y = waypoints[i, 1]
        traj_single.position.z = TRAJ_ALTITUDE
        traj_single.yaw = - (0.005236 * (i + 1))
        traj_single.timeMilliseconds = 3000 + (50 * (i + 1))
        # time_ms - 1 second/point

        circle_traj.trajectory.append(traj_single)

    # Publish the trajectory
    rospy.loginfo('Publishing trajectory')
    circle_traj.clearQueue = clear_queue
    circle_traj.header.stamp = rospy.Time.now()
    traj_pub.publish(circle_traj)
    rospy.loginfo('Done!')
