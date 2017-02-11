cmake_minimum_required(VERSION 2.8.3)
project(mir_description)

find_package(catkin REQUIRED COMPONENTS
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

#############
## Install ##
#############

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY meshes 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY urdf 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY rviz 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
