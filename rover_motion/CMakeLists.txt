cmake_minimum_required(VERSION 3.8)
project(rover_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ── IMU Node ──────────────────────────────────────────────────────────────────
add_executable(imu_node src/imu_node.cpp)

target_include_directories(imu_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(imu_node PUBLIC cxx_std_17)

ament_target_dependencies(imu_node
  rclcpp
  sensor_msgs
  std_msgs
)

# Note: i2c_smbus_* are inline helpers in <linux/i2c-dev.h>; no -li2c needed.

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS imu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config include
  DESTINATION share/${PROJECT_NAME}
)

# ── Linting (optional) ────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
