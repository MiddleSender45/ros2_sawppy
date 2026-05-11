cmake_minimum_required(VERSION 3.8)
project(rover_imu)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(imu_node src/imu_node.cpp)

ament_target_dependencies(imu_node
  rclcpp
  sensor_msgs
)

install(TARGETS imu_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()