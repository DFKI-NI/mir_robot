cmake_minimum_required(VERSION 2.8.3)
project(mir100_driver)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  diagnostic_msgs
  dynamic_reconfigure
  geometry_msgs
  mirMsgs
  mir_actions
  move_base_msgs
  nav_msgs
  rosgraph_msgs
  rospy
  rospy_message_converter
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    diagnostic_msgs
    dynamic_reconfigure
    geometry_msgs
    mirMsgs
    mir_actions
    move_base_msgs
    nav_msgs
    rosgraph_msgs
    sensor_msgs
    std_msgs
    tf
    visualization_msgs
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS nodes/mir100_bridge.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
