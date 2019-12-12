#!/usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from aabm_comms.msg import Waypoint, Trajectory_sngl, Trajectory
import math


class TrajectoryTests():
    def __init__(self):

        # Make publisher
        rospy.init_node('trajectory_tests', anonymous=True)
        self.rate = rospy.Rate(1) # 1hz - only used for countdown

        self.trajectory_publisher = rospy.Publisher('/ucl_0/autopilot/TrajectoryReference', Trajectory, queue_size=10)

        self.odom_topic = '/ucl_0/vrpn_client/estimated_odometry' #the topic that publishes the drone's whereabouts

        self.clear_queue = False #as we do not need to clear the previous trajectory in this case


    def run(self):

        rospy.logwarn('Starting Trajectory Tests')

        start = 0
        end = 1


        # while not shutting down
        while not rospy.is_shutdown() and start < end:

            new_trajectory = Trajectory() #creating new_trajectory to store the trajectory

            this_position = rospy.wait_for_message(self.odom_topic, Odometry) #getting the current position of the drone

            #x_pos = 0.0
            #y_pos = 3.0
            z_pos = 0.5

            #Setting the initial waypoint which should be the current position.
            waypoint = Waypoint()
            waypoint.position.x = this_position.pose.pose.position.x
            waypoint.position.y = this_position.pose.pose.position.y
            waypoint.position.z = this_position.pose.pose.position.z
            new_trajectory.initialWaypoint = waypoint

            #We start the trajectory with the 0th point being the same as the initial waypoint.
            traj_single = Trajectory_sngl() #creating traj_single to store every single waypoint

            traj_single.position.x = waypoint.position.x
            traj_single.position.y = waypoint.position.y
            traj_single.position.z = waypoint.position.z
            traj_single.timeMilliseconds = 0 #time_milliseconds

            new_trajectory.trajectory.append(traj_single) #append each single trajectory point to the entire trajectory.


            #Issuing a point for it to go to the start point, to ensure it stays same always
            traj_single = Trajectory_sngl() #needs to be defined each time for new waypoint

            traj_single.position.x = 0.0
            traj_single.position.y = 0.0
            traj_single.position.z = z_pos
            traj_single.yaw = 0
            traj_single.timeMilliseconds = 3000 #time_milliseconds

            new_trajectory.trajectory.append(traj_single)

            #zeroth point [0] ,first point [1] and initial point have been set
            #Now the second point and so on
            #We load the csv with the waypoints
            wp_csv = np.loadtxt('/home/aml/ucl_ws/src/example_trajectory_publisher/src/circle_waypoints_2_5_1200.csv', delimiter=",")


            for i in range(1200):
                traj_single = Trajectory_sngl()

                traj_single.position.x = wp_csv[i][0]
                traj_single.position.y = wp_csv[i][1]
                traj_single.position.z = z_pos
                traj_single.yaw = - (0.005236 * (i + 1))
                traj_single.timeMilliseconds = 3000 + (50 * (i + 1)) #time_milliseconds - 1 second/point

                new_trajectory.trajectory.append(traj_single)


            rospy.logwarn('Waiting for initialisation') #wait 5 sec to allow the system to initialise


            for i in range(5):
                rospy.logwarn(5 - i)
                self.rate.sleep()

            rospy.logwarn('Lets start') #issue the trajectory now

            new_trajectory.clearQueue = self.clear_queue

            new_trajectory.header.stamp = rospy.Time.now()

            self.trajectory_publisher.publish(new_trajectory)

            #countdown to finish the trajectory
            for i in range(3):
                rospy.logwarn(3 - i)
                self.rate.sleep()

            rospy.logwarn('Reached starting position. Beginning trajectory.')

            for i in range(60):
                rospy.logwarn(60 - i)
                self.rate.sleep()

            rospy.logwarn('End')


            start = end #to exit the program


if __name__ == '__main__':
    try:
        TrajectoryTests().run()
    except rospy.ROSInterruptException:
        pass
