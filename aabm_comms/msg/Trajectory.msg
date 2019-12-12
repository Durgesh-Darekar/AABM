Header header 

# initialWaypoint.position should coincide with trajectory[0].position
Waypoint initialWaypoint

# A full trajectory is a sequence of position, velocity, acceleration setpoints, yaw.
Trajectory_sngl[] trajectory

# When true, the Command queue is emptied and the current command is executed immediately
bool clearQueue
