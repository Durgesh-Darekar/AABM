#!/usr/bin/env python
from math import pi
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
TRAJ_DIAMENTER = 1.5 + 0.5
TRAJ_ALTITUDE = 0.5
TRAJ_REF_TOPIC = '/ucl_0/autopilot/TrajectoryReference'
ODOM_TOPIC = '/ucl_0/vrpn_client/estimated_odometry'

# Generates a circle trajectory. The number of waypoints is defined in
# `nb_waypoints` with a `diameter` and `altitude` in meters. The trajectory
# will start from a position defined by `start_pos` (x, y) in the world frame,
# and traverse the circle trajectory from the 6 o'clock position in a clock
# wise fashion.
#
# Returns: a np.array of waypoints (x[m], y[m], z[m], yaw[rads], time[ms])
def generate_circle_trajectory(nb_waypoints, diameter, altitude, start_pos):
    # Circle properties
    radius = diameter / 2.0

    # Generate waypoints
    index = 0
    waypoints = np.zeros((nb_waypoints, 5))  # x[m], y[m], z[m], yaw[rads], time[ms]
    for i in np.linspace(0.0, 1.0, num=nb_waypoints):
        # Calculate x and y position
        x = radius * cos(2.0 * pi * i + pi)
        y = radius * sin(2.0 * pi * i)

        # Calculate yaw and wrap it to be between pi and -pi
        yaw = 2.0 * pi * i
        if yaw > pi:
            yaw -= 2.0 * pi

        # Append to waypoints
        waypoints[index, 0] = start_pos[0] + radius + x
        waypoints[index, 1] = start_pos[1] + y
        waypoints[index, 2] = altitude
        waypoints[index, 3] = -1.0 * yaw  # Multiply by -1 to yaw clockwise.
        index += 1

    return waypoints


# Trajectory configuration.
# - nb_waypoints: Number of waypoints
# - diameter: Circle diameter [m]
# - altitude: Trajectory altitude [m]
# - start_pos: Starting position in the world frame (x, y) [m]
class TrajConfig:
    def __init__(self, **kwargs):
        self.nb_waypoints = kwargs.get("nb_waypoints", 1200)
        self.diameter = kwargs.get("diameter", 2.0)
        self.altitude = kwargs.get("altitude", 1.0)
        self.start_pos = kwargs.get("start_pos", [0.0, 0.0])


# Create trajectory message.
def create_traj_msg(drone_odom, traj_config, debug=False):
    # Generate scan trajectory
    traj = Trajectory()
    traj.clearQueue = True
    traj.header.stamp = 0 if debug else rospy.Time.now()

    # -- Setting initial waypoint which should be the drone's current position.
    wp_init = Waypoint()
    wp_init.position.x = drone_odom.pose.pose.position.x
    wp_init.position.y = drone_odom.pose.pose.position.y
    wp_init.position.z = drone_odom.pose.pose.position.z
    traj.initialWaypoint = wp_init

    # -- Zero-th point (same as the initial waypoint).
    wp_0 = Trajectory_sngl()
    wp_0.position.x = wp_init.position.x
    wp_0.position.y = wp_init.position.y
    wp_0.position.z = wp_init.position.z
    wp_0.yaw = 0
    wp_0.timeMilliseconds = 0
    traj.trajectory.append(wp_0)

    # -- First waypoint
    wp_1 = Trajectory_sngl()
    wp_1.position.x = traj_config.start_pos[0]
    wp_1.position.y = traj_config.start_pos[1]
    wp_1.position.z = traj_config.altitude
    wp_1.yaw = 0
    wp_1.timeMilliseconds = 3000  # Give MAV 3s to reach start position
    traj.trajectory.append(wp_1)

    # -- Generate other trajectory waypoints
    waypoints = generate_circle_trajectory(traj_config.nb_waypoints,
                                           traj_config.diameter,
                                           traj_config.altitude,
                                           traj_config.start_pos)
    for i in range(traj_config.nb_waypoints):
        wp = Trajectory_sngl()
        wp.position.x = waypoints[i, 0]
        wp.position.y = waypoints[i, 1]
        wp.position.z = waypoints[i, 2]
        wp.yaw = waypoints[i, 3]
        wp.timeMilliseconds = 3000 + (50 * (i + 1))
        traj.trajectory.append(wp)

    return traj


def test_generate_circle_trajectory():
    import matplotlib.pylab as plt
    from mpl_toolkits import mplot3d

    start_pos = [0.0, 0.0]
    waypoints = generate_circle_trajectory(NB_WAYPOINTS,
                                           TRAJ_DIAMENTER,
                                           TRAJ_ALTITUDE,
                                           start_pos)

    # Visualize
    plt.figure()
    plt.subplot(211)
    plt.plot(waypoints[:, 0], waypoints[:, 1])
    plt.title("x-y trajectory")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.subplot(212)
    plt.plot(range(NB_WAYPOINTS), waypoints[:, 3])
    plt.title("yaw")
    plt.xlabel("i-th waypoint")
    plt.ylabel("yaw [radians]")

    plt.figure()
    plt.plot(range(NB_WAYPOINTS), waypoints[:, 0], "r-", label="x")
    plt.plot(range(NB_WAYPOINTS), waypoints[:, 1], "g-", label="y")
    plt.legend()
    plt.xlabel("i-th waypoint")
    plt.ylabel("Displacement [m]")

    # fig = plt.figure()
    # ax = fig.gca(projection='3d')
    # ax.plot3D(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2])
    # ax.set_xlabel("x [m]")
    # ax.set_ylabel("y [m]")
    # ax.set_zlabel("z [m]")

    plt.show()


def test_create_traj_msg():
    import matplotlib.pylab as plt

    drone_odom = Odometry()
    clear_queue = True
    traj_config = TrajConfig()
    traj_config.nb_waypoints = 120
    traj_msg = create_traj_msg(drone_odom, clear_queue, traj_config, True)

    waypoints = np.zeros((len(traj_msg.trajectory), 4))
    index = 0
    for wp in traj_msg.trajectory:
        waypoints[index, 0] = wp.position.x
        waypoints[index, 1] = wp.position.y
        waypoints[index, 2] = wp.position.z
        waypoints[index, 3] = wp.yaw
        index += 1

    # Visualize
    plt.figure()

    plt.subplot(211)
    plt.plot(waypoints[:, 0], waypoints[:, 1])
    plt.title("x-y trajectory")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")

    plt.subplot(212)
    plt.plot(range(len(traj_msg.trajectory)), waypoints[:, 3])
    plt.title("yaw")
    plt.xlabel("i-th waypoint")
    plt.ylabel("yaw [radians]")

    plt.show()


if __name__ == '__main__':
    # # TESTS
    # test_generate_circle_trajectory()
    # test_create_traj_msg()

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

    # Get the curent drone's odometry
    rospy.loginfo('Waiting for drone odom msg')
    drone_odom = rospy.wait_for_message(ODOM_TOPIC, Odometry)

    # Publish trajectory
    rospy.loginfo('Publishing trajectory')
    traj_config = TrajConfig()
    scan_traj = create_traj_msg(drone_odom, traj_config)
    traj_pub.publish(scan_traj)

    # Wait 5 secs to allow the ros node to shutdown
    rospy.loginfo('Shutting down ros node ...')
    for i in range(5):
        rospy.loginfo(5 - i)
        node_rate.sleep()
    rospy.loginfo('Done!')
