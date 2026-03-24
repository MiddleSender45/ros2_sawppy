cmake_minimum_required(VERSION 3.5)
project(rover_navigation)

# Find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch params behavior_trees config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/waypoint_navigator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME waypoint_navigator
)

ament_package()