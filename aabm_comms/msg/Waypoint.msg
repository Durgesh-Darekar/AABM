# Reference MAV position expressed in World frame (units in meters) 
geometry_msgs/Vector3 position

# Reference yaw(psi) angle in radians 
float64		      yaw

# When norm ( Position_MAV  -  position) <= positionTolerance, the waypoint is considered “complete” and the MAV moves to the next waypoint
float64               positionTolerance
