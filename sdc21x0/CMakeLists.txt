cmake_minimum_required(VERSION 3.5.1)
project(sdc21x0)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Encoders.msg
  MotorCurrents.msg
  StampedEncoders.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  Flags.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)
