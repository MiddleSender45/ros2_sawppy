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
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch import conditions
from launch.actions import ExecuteProcess


def generate_launch_description():
    rover_bringup_shared_dir = get_package_share_directory("rover_bringup")
    rover_motor_controller_shared_dir = get_package_share_directory(
        "rover_motor_controller_cpp"
    )
    rover_teleop_shared_dir = get_package_share_directory("rover_teleop")

    env1 = SetEnvironmentVariable("RCUTILS_LOGGING_USE_STDOUT", "1")
    env2 = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")
    env3 = SetEnvironmentVariable("ROS_DISCOVERY_SERVER", "127.0.0.1:11811")
    env4 = SetEnvironmentVariable("RMW_MIDDLEWARE", "rmw_fastrtps_cpp")

    #
    # LAUNCHES
    #

    # Discovery server
    discovery_server_arg = DeclareLaunchArgument(
        "use_discovery_server",
        default_value="true",
        description="Whether to launch a local FastDDS discovery server",
    )
    use_discovery_server = LaunchConfiguration("use_discovery_server")

    ros_discovery_server_addr_arg = DeclareLaunchArgument(
        "ros_discovery_server_addr",
        default_value="127.0.0.1:11811",
        description="Value for ROS_DISCOVERY_SERVER to inject into nodes",
    )
    ros_discovery_server_addr = LaunchConfiguration("ros_discovery_server_addr")

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

    # Kinect
    use_kinect_arg = DeclareLaunchArgument(
        "use_kinect",
        default_value="true",
        description="Whether to launch the Kinect node",
    )
    use_kinect = LaunchConfiguration("use_kinect")

    kinect_pkg_share = get_package_share_directory("kinect_ros2")
    kinect_node_action_cmd = Node(
        package="kinect_ros2",
        executable="kinect_ros2_node",
        namespace="kinect",
        condition=conditions.IfCondition(use_kinect),
<<<<<<< HEAD
=======
        additional_env={
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
            "ROS_DISCOVERY_SERVER": ros_discovery_server_addr,
        },
>>>>>>> fork/working
    )

    launch_discovery_server = ExecuteProcess(
        cmd=["fastdds", "discovery", "-i", "0", "-l", "0.0.0.0", "-p", "11811"],
        name="fastdds_discovery_server",
        condition=conditions.IfCondition(use_discovery_server),
        output="screen",
    )

    teleop_twist_joy_action_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_teleop_shared_dir, "launch", "joy_teleop.launch.py")
        ),
        launch_arguments={"ros_discovery_server": ros_discovery_server_addr}.items(),
    )

    rover_motor_controller_action_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                rover_motor_controller_shared_dir, "launch", "motor_controller.launch.py"
            )
        ),
        launch_arguments={"ros_discovery_server": ros_discovery_server_addr}.items(),
    )

    ld = LaunchDescription()

    # Set up environment
    ld.add_action(env1)
    ld.add_action(env2)
    ld.add_action(env3)
    ld.add_action(env4)

    # Declare launch arguments so configurations exist at runtime
    ld.add_action(use_lidar_arg)
    ld.add_action(use_kinect_arg)
    ld.add_action(discovery_server_arg)
    ld.add_action(ros_discovery_server_addr_arg)

    # The included launch descriptions already have conditions attached
    # (conditions.IfCondition on the substitutions). Add them to the
    # launch description unconditionally and let the launch system
    # evaluate the conditions at runtime.
    ld.add_action(urg_node_action_cmd)
    ld.add_action(kinect_node_action_cmd)
    ld.add_action(teleop_twist_joy_action_cmd)
    ld.add_action(rover_motor_controller_action_cmd)
    ld.add_action(launch_discovery_server)

    return ld
