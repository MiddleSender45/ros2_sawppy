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

# This launch file is intended to be used on a different computer that the raspberry pi
# that is controlling Sawppy.  It may be on a different subnet - if so, you need the 
# discovery server running somewhere (I set it up to run on the pi)

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
    rover_teleop_shared_dir = get_package_share_directory("rover_teleop")
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_USE_STDOUT", "1"
    )
    stdout_linebuf2_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    # discovery server launch arg and environment
    ros_discovery_server_arg = DeclareLaunchArgument(
        "ros_discovery_server",
        default_value="127.0.0.1:11811",
        description="Value for ROS_DISCOVERY_SERVER to inject into teleop nodes",
    )
    ros_discovery_server = LaunchConfiguration("ros_discovery_server")

    rmw_envvar = SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_fastrtps_cpp")
    discovery_server_envvar = SetEnvironmentVariable(name="ROS_DISCOVERY_SERVER", value=ros_discovery_server)


    #
    # LAUNCHES
    #

    teleop_twist_joy_action_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_teleop_shared_dir, "launch", "joy_teleop.launch.py")
        )
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(stdout_linebuf2_envvar)
    ld.add_action(ros_discovery_server_arg)
    ld.add_action(rmw_envvar)
    ld.add_action(discovery_server_envvar)

    # The included launch descriptions already have conditions attached
    # (conditions.IfCondition on the substitutions). Add them to the
    # launch description unconditionally and let the launch system
    # evaluate the conditions at runtime.
    ld.add_action(teleop_twist_joy_action_cmd)

    return ld
