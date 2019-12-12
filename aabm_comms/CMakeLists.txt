cmake_minimum_required(VERSION 2.8.3)
project(aabm_comms)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

# Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Waypoint.msg
   Trajectory_sngl.msg
   Path.msg   
   Trajectory.msg
   HL_command.msg
   AutopilotStatus.msg
)

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  
   geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime
)
