cmake_minimum_required(VERSION 3.0.2)
project(costmap_erase_plugin)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  pluginlib
  roscpp
  nav_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES costmap_erase_plugin
  CATKIN_DEPENDS costmap_2d pluginlib roscpp nav_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/costmap_erase_plugin.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(PROGRAMS scripts/clear_costmap_timer_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
