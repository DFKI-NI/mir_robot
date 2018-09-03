cmake_minimum_required(VERSION 2.8.3)
project(mir_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
    Encoders.msg
    ExternalRobot.msg
    ExternalRobots.msg
    LocalMapStat.msg
    MirExtra.msg
    MirStatus.msg
    MissionCtrlCommand.msg
    MissionCtrlState.msg
    Path.msg
    Pose2D.msg
    RobotMode.msg
    RobotState.msg
    Serial.msg
    StampedEncoders.msg
    Trolley.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs
)
