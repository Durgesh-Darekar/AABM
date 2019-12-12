Header header
uint8 activeTask 
uint8 trajectoryCmdMode

# Active task enum
uint8 LANDED          = 1
uint8 FLYING          = 2
uint8 TAKING_OFF      = 3
uint8 LANDING         = 4
uint8 RETURN2HOME     = 5

# Trajectory Mode
uint8 IDLE             = 6
uint8 INITIAL_WAYPOINT = 7
uint8 TRAJECTORY       = 8
uint8 WAYPOINT         = 9
uint8 UNDEFINED        = 10
