#!/usr/bin/env python
# Sebs code, don't touch

import numpy as np

import rospy

from nav_msgs.msg import Odometry

from aabm_comms.msg import Trajectory

from trajectory_utility import *

import pandas as pd


class TrajectoryTests():
    def __init__(self):        
        # Make publisher
        rospy.init_node('trajectory_tests', anonymous=True)
        self.rate = rospy.Rate(1) # 1hz        
        self.trajectory_publisher = rospy.Publisher('/ucl_0/autopilot/TrajectoryReference', Trajectory, queue_size=10)

        self.trajectory_utility = TrajectoryUtility()
        self.trajectory_utility.set_max_speed(0.5)

        self.odom_topic = '/ucl_0/vrpn_client/estimated_odometry'

        self.yaw_tested = False

        self.travel_speed = 0.1

        self.pi_by_4 = 1.57079632679*2

        self.arrive_tolerance = 0.15

        self.target_yaw = 0

        this_position = rospy.wait_for_message(self.odom_topic, Odometry)
        self.current_target = np.array([this_position.pose.pose.position.x, this_position.pose.pose.position.y, this_position.pose.pose.position.z])

        self.clear_queue = False

    def send_to(self):
        rospy.logwarn('sending trajectory')

        this_position = rospy.wait_for_message(self.odom_topic, Odometry)
        
        start_position = np.array([this_position.pose.pose.position.x, this_position.pose.pose.position.y, this_position.pose.pose.position.z])
        end_position = self.current_target

        new_trajectory = self.trajectory_utility.go_to(start_position, end_position, self.travel_speed)

        self.trajectory_publisher.publish(new_trajectory)

    def send_yaw(self, yaw):
        if self.yaw_tested:
            rospy.logwarn('sending trajectory with yaw')

            this_position = rospy.wait_for_message(self.odom_topic, Odometry)

            start_position = np.array([this_position.pose.pose.position.x, this_position.pose.pose.position.y, this_position.pose.pose.position.z])
            end_position = self.current_target

            new_trajectory = self.trajectory_utility.go_to_yaw(start_position, end_position, self.target_yaw, yaw, self.travel_speed)

            self.target_yaw = yaw

            self.trajectory_publisher.publish(new_trajectory)
        else:
            rospy.logwarn('Have you tested that yaw works?')

    def am_i_there_yet(self):
        tolerance = self.arrive_tolerance
        this_position = rospy.wait_for_message(self.odom_topic, Odometry)

        current_position = np.array([this_position.pose.pose.position.x, this_position.pose.pose.position.y, this_position.pose.pose.position.z])
        delta_position = np.subtract(current_position, self.current_target)

        distance = np.linalg.norm(delta_position)

        if distance < tolerance:
            return True
        else:
            return False

    def run(self):
        # The goal here is to test various trajectoies:
        # Forward, backward, left, right, up, down, yaw, and a final more complex one
        rospy.logwarn('Starting Trajectory Tests')

        current_time = 0
        end_time = 15


        # while not shutting down
        while not rospy.is_shutdown() and current_time < end_time:

            #while current_time < end_time:

                #rospy.logwarn(current_time)
                
                #time_milliseconds = current_time * 1000    

                #new_trajectory.trajectory[current_time].position.x = x_pos
                #new_trajectory.trajectory[current_time].position.y = y_pos
                #new_trajectory.trajectory[current_time].position.z = 0.5
                #new_trajectory.trajectory[current_time].timeMilliseconds = time_milliseconds
            

            new_trajectory = Trajectory()
            traj_single = Trajectory_sngl()
            
            this_position = rospy.wait_for_message(self.odom_topic, Odometry)
            
            #x_pos = 0.0
            #y_pos = 3.0
            z_pos = 1.0
            waypoint = Waypoint()
            waypoint.position.x = this_position.pose.pose.position.x
            waypoint.position.y = this_position.pose.pose.position.y
            waypoint.position.z = this_position.pose.pose.position.z
            new_trajectory.initialWaypoint = waypoint

            traj_single.position.x = waypoint.position.x
            traj_single.position.y = waypoint.position.y
            traj_single.position.z = waypoint.position.z
            traj_single.timeMilliseconds = 0 #time_milliseconds

            new_trajectory.trajectory.append(traj_single)

            #zeroth point [0] and initial point have been set
            #Now the first point and so on
            #We load the csv with the waypoints
            wp_csv = np.loadtxt('/home/aml/ucl_ws/src/example_trajectory_publisher/src/circle_waypoints_2_0.csv', delimiter=",")
            
            
            for i in range(120):
                traj_single = Trajectory_sngl()

                traj_single.position.x = wp_csv[i][0]
                traj_single.position.y = wp_csv[i][1]
                traj_single.position.z = z_pos
                traj_single.yaw = - (0.05236 * (i + 1))
                traj_single.timeMilliseconds = 1000 * (i + 1) #time_milliseconds - 1 second/point

                new_trajectory.trajectory.append(traj_single)

           
            rospy.logwarn('Waiting for initialisation')
            

            for i in range(5):
                rospy.logwarn(5 - i)
                self.rate.sleep()

            rospy.logwarn('Lets start')
            new_trajectory.clearQueue = self.clear_queue

            new_trajectory.header.stamp = rospy.Time.now()
            
            self.trajectory_publisher.publish(new_trajectory)

            for i in range(120):
                rospy.logwarn(120 - i)
                self.rate.sleep()
            rospy.logwarn('End')
            current_time = end_time


if __name__ == '__main__':
    try:
        TrajectoryTests().run()
    except rospy.ROSInterruptException:
        pass
