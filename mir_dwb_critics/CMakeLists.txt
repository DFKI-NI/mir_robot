cmake_minimum_required(VERSION 3.5.1)
project(mir_dwb_critics)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Werror")

# The critics score ~1000 trajectories per control cycle; an unoptimized build misses the controller rate.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(catkin REQUIRED COMPONENTS
  angles
  costmap_queue
  dwb_critics
  dwb_local_planner
  geometry_msgs
  nav_2d_msgs
  nav_2d_utils
  nav_core2
  nav_grid_iterators
  pluginlib
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS angles costmap_queue dwb_critics dwb_local_planner geometry_msgs nav_2d_msgs nav_2d_utils nav_core2 nav_grid_iterators pluginlib roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/braking_obstacle_footprint.cpp
  src/path_angle.cpp
  src/path_progress.cpp
  src/path_follower.cpp
  src/path_dist_pruned.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(path_follower_test test/path_follower.test test/path_follower_test.cpp)
  target_link_libraries(path_follower_test ${PROJECT_NAME} ${catkin_LIBRARIES})
  # Debug tool: replay recorded poses against a recorded plan (see test/replay_path_follower.cpp).
  add_executable(replay_path_follower EXCLUDE_FROM_ALL test/replay_path_follower.cpp)
  target_link_libraries(replay_path_follower ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

install(PROGRAMS
    nodes/print_dwb_scores.py
    nodes/plot_dwb_scores.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES default_critics.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
