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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch import conditions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Package directories ────────────────────────────────────────────────
    rover_bringup_shared_dir      = get_package_share_directory("rover_bringup")
    rover_motor_controller_dir    = get_package_share_directory("rover_motor_controller_cpp")
    robot_description_pkg_share   = get_package_share_directory("rover_description")
    pkg_rover_navigation          = get_package_share_directory("rover_navigation")

    # ── Launch arguments ───────────────────────────────────────────────────

    # LIDAR
    use_lidar_arg = DeclareLaunchArgument(
        "use_lidar",
        default_value="true",
        description="Whether to launch the urg_node (LIDAR driver)",
    )
    use_lidar = LaunchConfiguration("use_lidar")

    # Robot State Publisher
    use_robot_state_publisher_arg = DeclareLaunchArgument(
        "use_robot_state_publisher",
        default_value="true",
        description="Whether to launch the robot_state_publisher node",
    )
    use_robot_state_publisher = LaunchConfiguration("use_robot_state_publisher")

    # Motor Controller
    use_motor_controller_arg = DeclareLaunchArgument(
        "use_motor_controller",
        default_value="true",
        description="Whether to launch the rover motor controller node",
    )
    use_motor_controller = LaunchConfiguration("use_motor_controller")

    # rf2o odometry
    use_rf2o_arg = DeclareLaunchArgument(
        "use_rf2o",
        default_value="true",
        description="Whether to launch the rf2o laser odometry node",
    )
    use_rf2o = LaunchConfiguration("use_rf2o")

    # Nav2 planner / controller selectors
    nav2_planner_cmd = DeclareLaunchArgument(
        "nav2_planner",
        default_value="SmacHybrid",
        choices=["SmacHybrid", "SmacLattice"],
        description="Nav2 planner (SmacHybrid or SmacLattice)",
    )
    nav2_planner = LaunchConfiguration("nav2_planner")

    nav2_controller_cmd = DeclareLaunchArgument(
        "nav2_controller",
        default_value="RPP",
        choices=["RPP", "TEB"],
        description="Nav2 controller (RPP or TEB)",
    )
    nav2_controller = LaunchConfiguration("nav2_controller")

    # ── 1. URG LIDAR ───────────────────────────────────────────────────────
    urg_node_cmd = IncludeLaunchDescription(
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

    # ── 2. Robot State Publisher ───────────────────────────────────────────
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                robot_description_pkg_share, "launch", "robot_state_publisher.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": "false"}.items(),
        condition=conditions.IfCondition(use_robot_state_publisher),
    )

    # ── 3. Motor Controller ────────────────────────────────────────────────
    rover_motor_controller_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                rover_motor_controller_dir, "launch", "motor_controller.launch.py"
            )
        ),
        condition=conditions.IfCondition(use_motor_controller),
    )

    # ── 4. rf2o Laser Odometry ─────────────────────────────────────────────
    # Parameters follow the standard rf2o_laser_odometry documentation.
    # Adjust laser_scan_topic / odom_topic to match your TF / topic names.
    rf2o_node_cmd = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[{
            "laser_scan_topic":   "/scan",          # topic published by urg_node
            "odom_topic":         "/odom_rf2o",     # outgoing odometry topic
            "publish_tf":         True,              # publish odom → base_link TF
            "base_frame_id":      "base_link",
            "odom_frame_id":      "odom",
            "init_pose_from_topic": "",
            "freq":               10.0,             # Hz
        }],
        condition=conditions.IfCondition(use_rf2o),
    )

    # ── 5. Navigation (Nav2) — delayed so sensors/odom are ready ──────────
    # Full Nav2 bringup at t = 10 s
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_navigation, "launch", "bringup.launch.py")
        ),
        launch_arguments={
            "use_sim_time":  "false",
            "planner":       nav2_planner,
            "controller":    nav2_controller,
        }.items(),
    )

    # ── Build LaunchDescription ────────────────────────────────────────────
    ld = LaunchDescription()

    # Arguments
    ld.add_action(use_lidar_arg)
    ld.add_action(use_robot_state_publisher_arg)
    ld.add_action(use_motor_controller_arg)
    ld.add_action(use_rf2o_arg)
    ld.add_action(nav2_planner_cmd)
    ld.add_action(nav2_controller_cmd)

    # Nodes / includes — launched immediately
    ld.add_action(urg_node_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(rover_motor_controller_cmd)
    ld.add_action(rf2o_node_cmd)

    # Nav2 delayed so sensors/odom are ready
    ld.add_action(TimerAction(period=10.0, actions=[navigation_cmd]))

    return ld