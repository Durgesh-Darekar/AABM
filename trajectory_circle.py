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
        angle = section_angle * i
        angle_rad = radians(angle)

        if 0 <= angle and angle <= 90:
            cos_x = radius * cos(angle)
            sin_y = radius * sin(angle)
            x_pos = x_start + (radius - cos_x)
            y_pos = y_start + sin_y
            waypoints[i, 0] = x_pos
            waypoints[i, 1] = y_pos

        elif angle <= 180:
            ang = angle - 90
            sin_x = radius * sin(ang)
            cos_y = radius * cos(ang)
            x_pos = x_start + (radius + sin_x)
            y_pos = y_start + cos_y
            waypoints[i, 0] = x_pos
            waypoints[i, 1] = y_pos

        elif angle <= 270:
            ang = angle - 180
            cos_x = radius * cos(ang)
            sin_y = radius * sin(ang)
            x_pos = x_start + (radius + cos_x)
            y_pos = y_start - sin_y
            waypoints[i, 0] = x_pos
            waypoints[i, 1] = y_pos

        elif angle <= 360:
            ang = angle - 270
            sin_x = radius * sin(ang)
            cos_y = radius * cos(ang)
            x_pos = x_start + (radius - sin_x)
            y_pos = y_start - cos_y
            waypoints[i, 0] = x_pos
            waypoints[i, 1] = y_pos

    return waypoints


class TrajectoryPublisher():
    def __init__(self):
        rospy.init_node('trajectory_circle', anonymous=True)
        self.rate = rospy.Rate(1)
        self.trajectory_publisher = rospy.Publisher(TRAJ_REF_TOPIC, Trajectory, queue_size=10)
        self.clear_queue = False # as we do not need to clear the previous trajectory in this case

    def run(self):
        rospy.logwarn('Starting Trajectory Tests')
        start = 0
        end = 1

        while not rospy.is_shutdown() and start < end:
            new_trajectory = Trajectory()
            this_position = rospy.wait_for_message(ODOM_TOPIC, Odometry) #getting the current position of the drone

            #x_pos = 0.0
            #y_pos = 3.0
            z_pos = TRAJ_ALTITUDE

            # Setting initial waypoint which should be the current position.
            waypoint = Waypoint()
            waypoint.position.x = this_position.pose.pose.position.x
            waypoint.position.y = this_position.pose.pose.position.y
            waypoint.position.z = this_position.pose.pose.position.z
            new_trajectory.initialWaypoint = waypoint

            # Start the trajectory with the 0th point being the same as the initial waypoint.
            traj_single = Trajectory_sngl()
            traj_single.position.x = waypoint.position.x
            traj_single.position.y = waypoint.position.y
            traj_single.position.z = waypoint.position.z
            traj_single.timeMilliseconds = 0
            new_trajectory.trajectory.append(traj_single)

            # Issuing a point for it to go to the start point, to ensure it stays same always
            traj_single = Trajectory_sngl()
            traj_single.position.x = 0.0
            traj_single.position.y = 0.0
            traj_single.position.z = z_pos
            traj_single.yaw = 0
            traj_single.timeMilliseconds = 3000
            new_trajectory.trajectory.append(traj_single)

            # Generate trajectory waypoints
            waypoints = generate_circle_trajectory(NB_WAYPOINTS, CIRCLE_DIAMETER)
            assert(waypoints.shape[0] == NB_WAYPOINTS)
            assert(waypoints.shape[1] == 2)

            for i in range(NB_WAYPOINTS):
                traj_single = Trajectory_sngl()
                traj_single.position.x = waypoints[i, 0]
                traj_single.position.y = waypoints[i, 1]
                traj_single.position.z = z_pos
                traj_single.yaw = - (0.005236 * (i + 1))
                traj_single.timeMilliseconds = 3000 + (50 * (i + 1)) # time_ms - 1 second/point
                new_trajectory.trajectory.append(traj_single)

            # Wait 5 secs to allow the system to initialise
            rospy.logwarn('Waiting for initialisation')
            for i in range(5):
                rospy.logwarn(5 - i)
                self.rate.sleep()

            # Start publishing trajectory
            rospy.logwarn('Lets start') #issue the trajectory now
            new_trajectory.clearQueue = self.clear_queue
            new_trajectory.header.stamp = rospy.Time.now()
            self.trajectory_publisher.publish(new_trajectory)

            # Countdown to start of trajectory
            for i in range(3):
                rospy.logwarn(3 - i)
                self.rate.sleep()

            rospy.logwarn('Reached starting position. Beginning trajectory.')
            for i in range(60):
                rospy.logwarn(60 - i)
                self.rate.sleep()

            rospy.logwarn('End')
            start = end # to exit the program


if __name__ == '__main__':
    # waypoints = generate_circle_trajectory(NB_WAYPOINTS, CIRCLE_DIAMETER)
    # import matplotlib.pylab as plt
    # plt.plot(waypoints[:, 0], waypoints[:, 1])
    # plt.show()

    try:
        TrajectoryPublisher().run()
    except rospy.ROSInterruptException:
        pass
