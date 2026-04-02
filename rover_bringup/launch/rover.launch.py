# MIT License

# Copyright (c) 2023 Miguel Ángel González Santamarta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch import conditions

def generate_launch_description():
    rover_bringup_shared_dir = get_package_share_directory("rover_bringup")
    rover_motor_controller_shared_dir = get_package_share_directory(
        "rover_motor_controller_cpp"
    )
    rover_teleop_shared_dir = get_package_share_directory("rover_teleop")

    env1 = SetEnvironmentVariable("RCUTILS_LOGGING_USE_STDOUT", "1")
    env2 = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    # Robot State Publisher
    use_robot_state_publisher_arg = DeclareLaunchArgument(
        "use_robot_state_publisher",
        default_value="true",
        description="Whether to launch the robot_state_publisher node",
    )
    use_robot_state_publisher = LaunchConfiguration("use_robot_state_publisher")

    robot_description_pkg_share = get_package_share_directory("rover_description")

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_description_pkg_share, "launch", "robot_state_publisher.launch.py")
        ),
        condition=conditions.IfCondition(use_robot_state_publisher),
    )

    # LIDAR
    use_lidar_arg = DeclareLaunchArgument(
        "use_lidar",
        default_value="false",
        description="Whether to launch the urg_node (LIDAR driver)",
    )
    use_lidar = LaunchConfiguration("use_lidar")

    urg_node_action_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_bringup_shared_dir, "launch", "urg_node.launch.py")
        ),
        launch_arguments={
            "config_filepath": os.path.join(
                rover_bringup_shared_dir, "config", "urg_node_serial.yaml"
            )
        }.items(),
        condition=conditions.IfCondition(use_lidar),
    )

    teleop_twist_joy_action_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_teleop_shared_dir, "launch", "joy_teleop.launch.py")
        ),
    )

    rover_motor_controller_action_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                rover_motor_controller_shared_dir, "launch", "motor_controller.launch.py"
            )
        ),
    )

    ld = LaunchDescription()

    ld.add_action(env1)
    ld.add_action(env2)

    ld.add_action(use_lidar_arg)
    ld.add_action(use_robot_state_publisher_arg)

    ld.add_action(urg_node_action_cmd)
    ld.add_action(teleop_twist_joy_action_cmd)
    ld.add_action(rover_motor_controller_action_cmd)
    ld.add_action(robot_state_publisher_launch)

    return ld