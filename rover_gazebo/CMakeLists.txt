cmake_minimum_required(VERSION 3.5)
project(rover_gazebo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rover_msgs REQUIRED)
find_package(rover_description REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

# Harmonic
message(STATUS "Compiling against Gazebo Harmonic")
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

# Following 'add_library' directive defines a library target named 'BasicSystem'.
# The 'SHARED' keyword indicates that a shared library should be compiled, and
# is followed by the list of source files for the target.
add_library(BasicSystem
  SHARED
  src/BasicSystem.cc
)

# Following 'target_include_directories' directive specifies 'include' as the
# include directory to use when compiling the 'BasicSystem' target.
# The 'PRIVATE' keyword specifies that this directive will populate the
# INCLUDE_DIRECTORIES property for the 'BasicSystem' target.
target_include_directories(
  BasicSystem PRIVATE include
)

# Following 'target_link_libraries' directive specifies to use the 
# gz-sim library when linking the 'BasicSystem' target.
# The 'PRIVATE' keyword stipulates that the gz-sim library will not
# automatically be included if the 'BasicSystem' target were to
# be linked to anoter target. 
# ${GZ_SIM_VER} is substituted by the value that is was set to above.
target_link_libraries(BasicSystem PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})


INCLUDE_DIRECTORIES(
  include
)

add_executable(motors_command_parser_node src/motors_command_parser_node.cpp)

target_link_libraries(motors_command_parser_node)
ament_target_dependencies(motors_command_parser_node 
  rclcpp
  rover_msgs
  std_msgs
  trajectory_msgs
)

install(TARGETS
  motors_command_parser_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch worlds models rviz config
  DESTINATION share/${PROJECT_NAME}
)

# Following hooks are used to ensure that the correct environment variables
# will be set by exectuting 'source install/setup.bash' after compilation.
# When using this template for your project, change the filenames of the
# files in the 'hooks' folder, to correspond to your project name.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
