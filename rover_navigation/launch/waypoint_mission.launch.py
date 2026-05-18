"""
waypoint_mission.launch.py
--------------------------
Launches ONLY the waypoint navigator node against an already-running
Forest map + Nav2 stack.  Does NOT start Gazebo or Nav2 itself.

Usage:
    ros2 launch <your_pkg> waypoint_mission.launch.py
    ros2 launch <your_pkg> waypoint_mission.launch.py \
        waypoints:=/absolute/path/to/waypoints_forest.yaml

Prerequisites (must already be running):
    - Gazebo with the forest world
    - robot_state_publisher / joint_state_publisher
    - Nav2 (bringup)
    - map_server with the forest map
    - AMCL or another localizer
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ----------------------------------------------------------------
    # Launch arguments
    # ----------------------------------------------------------------
    default_waypoints = PathJoinSubstitution([
        FindPackageShare("rover_navigation"),   # <-- replace with your package name
        "config",
        "waypoints_indoor.yaml",
    ])

    waypoints_arg = DeclareLaunchArgument(
        "waypoints",
        default_value=default_waypoints,
        description="Absolute path to the waypoints YAML file",
    )

    # ----------------------------------------------------------------
    # Waypoint navigator node
    # ----------------------------------------------------------------
    waypoint_navigator_node = Node(
        package="rover_navigation",             # <-- replace with your package name
        executable="waypoint_navigator",
        name="waypoint_navigator",
        output="screen",
        emulate_tty=True,
        arguments=["--waypoints", LaunchConfiguration("waypoints")],
        parameters=[{
            "use_sim_time": False,       # set False if running on real hardware
        }],
    )

    return LaunchDescription([
        waypoints_arg,
        LogInfo(msg="--- Waypoint Mission Launch ---"),
        LogInfo(msg=["Using waypoints file: ", LaunchConfiguration("waypoints")]),
        waypoint_navigator_node,
    ])
