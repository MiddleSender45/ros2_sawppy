cmake_minimum_required(VERSION 3.5)
project(rover_bringup)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rover_description REQUIRED)
find_package(rover_gazebo REQUIRED)


# install the launch directory
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
