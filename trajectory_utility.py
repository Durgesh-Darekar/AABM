#!/usr/bin/env python
# Seb's code - don't touch

import numpy as np

import rospy
from geometry_msgs.msg import TransformStamped
from aabm_comms.msg import Waypoint, Trajectory_sngl, Trajectory
import math

class TrajectoryUtility():
  def __init__(self):

    self.max_speed = 0.2
    self.max_acceleration = 0.1 #unused
    self.find_yaw = False
    self.clear_queue = False

  # Some quaternion utility functions
  def quaternion_mult(self, q, r):
    return [r[0] * q[0] - r[1] * q[1] - r[2] * q[2] - r[3] * q[3],
            r[0] * q[1] + r[1] * q[0] - r[2] * q[3] + r[3] * q[2],
            r[0] * q[2] + r[1] * q[3] + r[2] * q[0] - r[3] * q[1],
            r[0] * q[3] - r[1] * q[2] + r[2] * q[1] + r[3] * q[0]]

  def point_rotation_by_quaternion(self, point, q):
    r = [0] + point
    q_conj = [q[0], -1 * q[1], -1 * q[2], -1 * q[3]]
    return self.quaternion_mult(self.quaternion_mult(q, r), q_conj)[1:]
  # End of quaternion functions

  def set_find_yaw(self, should_i_find_yaw):
    self.find_yaw = should_i_find_yaw

  def set_max_speed(self, new_speed):
    if new_speed < 0.01:
      new_speed = 0.01
      rospy.logwarn('VERY LOW MAX SPEED SET.')

    self.max_speed = new_speed

  def set_clear_queue(self, should_i_clear_queue):
    self.clear_queue = should_i_clear_queue

  def go_to(self, current_position, end_position, flight_speed):
    new_trajectory = Trajectory()
    current_time = 0

    if flight_speed > self.max_speed:
      rospy.logwarn('Given speed for trajectory is greater than set max speed.')
      flight_speed = self.max_speed

    # initial waypoint
    waypoint = Waypoint()
    waypoint.position.x = current_position[0]
    waypoint.position.y = current_position[1]
    waypoint.position.z = current_position[2]

    # some default values - I plan on changing these later
    waypoint.yaw = 0
    waypoint.positionTolerance = 0.05

    new_trajectory.initialWaypoint = waypoint

    # final waypoint
    trajectory_single_start = Trajectory_sngl()
    trajectory_single_start.position.x = current_position[0]
    trajectory_single_start.position.y = current_position[1]
    trajectory_single_start.position.z = current_position[2]

    current_time = current_time
    trajectory_single_start.timeMilliseconds = np.round(current_time)

    new_trajectory.trajectory.append(trajectory_single_start)

    trajectory_single = Trajectory_sngl()
    trajectory_single.position.x = end_position[0]
    trajectory_single.position.y = end_position[1]
    trajectory_single.position.z = end_position[2]

    previous_position = np.array([waypoint.position.x, waypoint.position.y, waypoint.position.z])

    current_position = np.array([trajectory_single.position.x,
                                 trajectory_single.position.y,
                                 trajectory_single.position.z
                                 ])

    delta_position = np.subtract(current_position, previous_position)
    distance = np.linalg.norm(delta_position)

    next_time = 1000 * distance / flight_speed
    current_time = current_time + next_time
    trajectory_single.timeMilliseconds = np.round(current_time)

    new_trajectory.trajectory.append(trajectory_single)

    new_trajectory.clearQueue = self.clear_queue

    new_trajectory.header.stamp = rospy.Time.now()

    return new_trajectory

  def go_to_yaw(self, current_position, end_position, start_yaw, end_yaw, flight_speed):
    # Yaw is in radians as an input
    new_trajectory = Trajectory()
    current_time = 0

    if flight_speed > self.max_speed:
      rospy.logwarn('Given speed for trajectory is greater than set max speed.')
      flight_speed = self.max_speed

    # initial waypoint
    waypoint = Waypoint()
    waypoint.position.x = current_position[0]
    waypoint.position.y = current_position[1]
    waypoint.position.z = current_position[2]

    # some default values - I plan on changing these later
    waypoint.yaw = start_yaw
    waypoint.positionTolerance = 0.05

    new_trajectory.initialWaypoint = waypoint

    # need to iterate through point to get to the end - maye I should always have like 10 points
    trajectory_single_start = Trajectory_sngl()
    trajectory_single_start.position.x = current_position[0]
    trajectory_single_start.position.y = current_position[1]
    trajectory_single_start.position.z = current_position[2]
    trajectory_single_start.yaw = start_yaw

    current_time = current_time
    trajectory_single_start.timeMilliseconds = np.round(current_time)

    new_trajectory.trajectory.append(trajectory_single_start)

    previous_position = np.array([waypoint.position.x, waypoint.position.y, waypoint.position.z])

    delta_position = np.subtract(current_position, end_position)
    distance = np.linalg.norm(delta_position)
    next_time = 1000 * distance / flight_speed

    delta_yaw = start_yaw - end_yaw

    inc_pos = delta_position * 0.1

    inc_yaw = delta_yaw * 0.1
    inc_time = next_time / 30
    inc_angle = 1.57 / 30

    #if inc_time < 10:
     # inc_time = 10

    for i in range(1, 30):
      trajectory_single = Trajectory_sngl()
      trajectory_single.position.x = current_position[0] - math.cos(inc_angle * i)
      trajectory_single.position.y = current_position[1] + math.sin(inc_angle * i)
      trajectory_single.position.z = current_position[2] + inc_pos[2] * i
      trajectory_single.yaw = start_yaw + inc_yaw

      current_time = current_time + inc_time
      trajectory_single.timeMilliseconds = np.round(current_time)

      new_trajectory.trajectory.append(trajectory_single)

    new_trajectory.clearQueue = self.clear_queue

    new_trajectory.header.stamp = rospy.Time.now()

    return new_trajectory

  def path_to_trajectory(self, path, start_time, find_yaw):
    # A trajectory single has:
    # Position
    # Velocity
    # Acceleration
    # yaw
    # time
    # I can send just positions and times

    new_trajectory = Trajectory()
    current_time = start_time

    for i, wp in enumerate(path.waypoints):
      trajectory_single = Trajectory_sngl()
      trajectory_single.position = wp.position

      if i == 0:
        trajectory_single.timeMilliseconds = current_time
        new_trajectory.initialWaypoint = wp
      else:
        previous_position = np.array([waypoint.position.x, waypoint.position.y, waypoint.position.z])
        current_position = np.array([trajectory_single.position.x, trajectory_single.position.y,
                                     trajectory_single.position.z])

        delta_position = np.subtract(current_position, previous_position)
        distance = np.linalg.norm(delta_position)

        next_time = 1000 * distance/self.printing_speed
        current_time = current_time + next_time
        trajectory_single.timeMilliseconds = np.round(current_time)

        if find_yaw:
          trajectory_single.yaw = wp.yaw

      new_trajectory.trajectory.append(trajectory_single)

    # At this point I have trajectories with timestamps
    # I now need velocity and acceleration
    # and yaw if i want it
    for i, t_s in enumerate(new_trajectory.trajectory):
      # need to account for first and last point
      dx = 0
      dy = 0
      dz = 0
      dt = 0

      if i == 0:
        dx = new_trajectory.trajectory[i + 1].position.x - new_trajectory.trajectory[i].position.x
        dy = new_trajectory.trajectory[i + 1].position.y - new_trajectory.trajectory[i].position.y
        dz = new_trajectory.trajectory[i + 1].position.z - new_trajectory.trajectory[i].position.z
        dt = new_trajectory.trajectory[i + 1].timeMilliseconds - new_trajectory.trajectory[i].timeMilliseconds

      elif i == len(new_trajectory.trajectory) - 1:
        dx = new_trajectory.trajectory[i].position.x - new_trajectory.trajectory[i - 1].position.x
        dy = new_trajectory.trajectory[i].position.y - new_trajectory.trajectory[i - 1].position.y
        dz = new_trajectory.trajectory[i].position.z - new_trajectory.trajectory[i - 1].position.z
        dt = new_trajectory.trajectory[i].timeMilliseconds - new_trajectory.trajectory[i - 1].timeMilliseconds

      else:
        # velocity = distance/time - also need vector
        dx = new_trajectory.trajectory[i + 1].position.x - new_trajectory.trajectory[i - 1].position.x
        dy = new_trajectory.trajectory[i + 1].position.y - new_trajectory.trajectory[i - 1].position.y
        dz = new_trajectory.trajectory[i + 1].position.z - new_trajectory.trajectory[i - 1].position.z
        dt = new_trajectory.trajectory[i + 1].timeMilliseconds - new_trajectory.trajectory[
          i - 1].timeMilliseconds

      # velocity = d/t
      vx = dx / dt
      vy = dy / dt
      vz = dz / dt

      new_trajectory.trajectory[i].linearVelocity.x = vx
      new_trajectory.trajectory[i].linearVelocity.y = vy
      new_trajectory.trajectory[i].linearVelocity.z = vz

    # I should now have velocity - now give acceleration

    for i, t_s in enumerate(new_trajectory.trajectory):
      # need to account for first and last point
      vx = 0
      vy = 0
      vz = 0
      dt = 0

      if i == 0:
        vx = new_trajectory.trajectory[i + 1].linearVelocity.x - new_trajectory.trajectory[i].linearVelocity.x
        vy = new_trajectory.trajectory[i + 1].linearVelocity.y - new_trajectory.trajectory[i].linearVelocity.y
        vz = new_trajectory.trajectory[i + 1].linearVelocity.z - new_trajectory.trajectory[i].linearVelocity.z
        dt = new_trajectory.trajectory[i + 1].timeMilliseconds - new_trajectory.trajectory[i].timeMilliseconds
      elif i == len(new_trajectory.trajectory) - 1:
        vx = new_trajectory.trajectory[i].linearVelocity.x - new_trajectory.trajectory[i - 1].linearVelocity.x
        vy = new_trajectory.trajectory[i].linearVelocity.y - new_trajectory.trajectory[i - 1].linearVelocity.y
        vz = new_trajectory.trajectory[i].linearVelocity.z - new_trajectory.trajectory[i - 1].linearVelocity.z
        dt = new_trajectory.trajectory[i].timeMilliseconds - new_trajectory.trajectory[i - 1].timeMilliseconds
      else:
        # velocity = distance/time - also need vector
        vx = new_trajectory.trajectory[i + 1].linearVelocity.x - new_trajectory.trajectory[
          i - 1].linearVelocity.x
        vy = new_trajectory.trajectory[i + 1].linearVelocity.y - new_trajectory.trajectory[
          i - 1].linearVelocity.y
        vz = new_trajectory.trajectory[i + 1].linearVelocity.z - new_trajectory.trajectory[
          i - 1].linearVelocity.z
        dt = new_trajectory.trajectory[i + 1].timeMilliseconds - new_trajectory.trajectory[
          i - 1].timeMilliseconds  # in milliseconds

      # velocity = d/t
      ax = vx / dt
      ay = vy / dt
      az = vz / dt

      new_trajectory.trajectory[i].linearAcceleration.x = ax
      new_trajectory.trajectory[i].linearAcceleration.y = ay
      new_trajectory.trajectory[i].linearAcceleration.z = az

    new_trajectory.clearQueue = self.clear_queue

    new_trajectory.header.stamp = rospy.Time.now()

    return new_trajectory
