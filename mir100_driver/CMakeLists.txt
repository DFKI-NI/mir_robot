cmake_minimum_required(VERSION 2.8.3)
project(mir100_driver)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  move_base_msgs
  nav_msgs
  rospy
  rospy_message_converter
  sensor_msgs
  tf
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    move_base_msgs
    nav_msgs
    sensor_msgs
    tf
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS nodes/mir100_bridge.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
