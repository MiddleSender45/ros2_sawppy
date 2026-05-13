# MIT License
# Copyright (c) 2023 Miguel Ángel González Santamarta

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
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Package directories ────────────────────────────────────────────────
    rover_bringup_shared_dir    = get_package_share_directory("rover_bringup")
    rover_motor_controller_dir  = get_package_share_directory("rover_motor_controller_cpp")
    robot_description_pkg_share = get_package_share_directory("rover_description")
    pkg_rover_navigation        = get_package_share_directory("rover_navigation")
    pkg_rover_localization      = get_package_share_directory("rover_localization")

    # ── Launch arguments ───────────────────────────────────────────────────

    use_lidar_arg = DeclareLaunchArgument(
        "use_lidar",
        default_value="true",
        description="Whether to launch the urg_node (LIDAR driver)",
    )
    use_lidar = LaunchConfiguration("use_lidar")

    use_robot_state_publisher_arg = DeclareLaunchArgument(
        "use_robot_state_publisher",
        default_value="true",
        description="Whether to launch the robot_state_publisher node",
    )
    use_robot_state_publisher = LaunchConfiguration("use_robot_state_publisher")

    use_motor_controller_arg = DeclareLaunchArgument(
        "use_motor_controller",
        default_value="true",
        description="Whether to launch the rover motor controller node",
    )
    use_motor_controller = LaunchConfiguration("use_motor_controller")

    use_rf2o_arg = DeclareLaunchArgument(
        "use_rf2o",
        default_value="true",
        description="Whether to launch the rf2o laser odometry node",
    )
    use_rf2o = LaunchConfiguration("use_rf2o")

    use_rtabmap_arg = DeclareLaunchArgument(
        "use_rtabmap",
        default_value="true",
        description="Whether to launch RTAB-Map SLAM (provides map->odom TF)",
    )
    use_rtabmap = LaunchConfiguration("use_rtabmap")

    # rtabmapviz is optional — useful on a desktop/laptop with a screen,
    # but disable it on a headless rover computer to save resources.
    launch_rtabmapviz_arg = DeclareLaunchArgument(
        "launch_rtabmapviz",
        default_value="false",
        description="Whether to launch rtabmapviz (disable on headless robot)",
    )
    launch_rtabmapviz = LaunchConfiguration("launch_rtabmapviz")

    nav2_planner_arg = DeclareLaunchArgument(
        "nav2_planner",
        default_value="SmacHybrid",
        choices=["SmacHybrid", "SmacLattice"],
        description="Nav2 planner (SmacHybrid or SmacLattice)",
    )
    nav2_planner = LaunchConfiguration("nav2_planner")

    nav2_controller_arg = DeclareLaunchArgument(
        "nav2_controller",
        default_value="RPP",
        choices=["RPP", "TEB"],
        description="Nav2 controller (RPP or TEB)",
    )
    nav2_controller = LaunchConfiguration("nav2_controller")

    # ── 1. URG LIDAR ───────────────────────────────────────────────────────
    # Launched immediately — everything else depends on /scan being available.
    urg_node_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_bringup_shared_dir, "launch", "urg_node.launch.py")
        ),
        launch_arguments={
            "config_filepath": os.path.join(
                rover_bringup_shared_dir, "config", "urg_node_serial.yaml"
            )
        }.items(),
        condition=IfCondition(use_lidar),
    )

    # ── 2. Robot State Publisher ───────────────────────────────────────────
    # Provides base_link → sensor TFs from URDF. Launched immediately.
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                robot_description_pkg_share, "launch", "robot_state_publisher.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": "false"}.items(),
        condition=IfCondition(use_robot_state_publisher),
    )

    # ── 3. Motor Controller ────────────────────────────────────────────────
    rover_motor_controller_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                rover_motor_controller_dir, "launch", "motor_controller.launch.py"
            )
        ),
        condition=IfCondition(use_motor_controller),
    )

    # ── 4. rf2o Laser Odometry  (delayed 3 s — waits for /scan) ───────────
    # Produces odom → base_link TF which RTAB-Map subscribes to via /odom.
    rf2o_node_cmd = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[{
            "laser_scan_topic":     "/scan",
            "odom_topic":           "/odom",      # NOTE: remapped to /odom so rtabmap
            "publish_tf":           True,          # receives it on the expected topic
            "base_frame_id":        "base_link",
            "odom_frame_id":        "odom",
            "init_pose_from_topic": "",
            "freq":                 10.0,
        }],
        condition=IfCondition(use_rf2o),
    )

    # ── 5. RTAB-Map SLAM  (delayed 6 s — waits for odom + /scan) ──────────
    # Produces map → odom TF; this is the missing link that Nav2 requires.
    # rtabmap.launch.py already handles the rtabmapviz conditional internally.
    rtabmap_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_localization, "launch", "rtabmap.launch.py")
        ),
        launch_arguments={
            "use_sim_time":       "false",
            "launch_rtabmapviz":  launch_rtabmapviz,
        }.items(),
        condition=IfCondition(use_rtabmap),
    )

    # ── 6. Nav2  (delayed 15 s — waits for map → odom TF from RTAB-Map) ───
    # RTAB-Map needs a few seconds after startup before it publishes a stable
    # map → odom transform. 15 s total (6 s rtabmap start + ~9 s settling)
    # is conservative but reliable.  Tune down once the stack is proven.
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_navigation, "launch", "bringup.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "planner":      nav2_planner,
            "controller":   nav2_controller,
        }.items(),
    )

    # ── Build LaunchDescription ────────────────────────────────────────────
    ld = LaunchDescription()

    # Arguments
    ld.add_action(use_lidar_arg)
    ld.add_action(use_robot_state_publisher_arg)
    ld.add_action(use_motor_controller_arg)
    ld.add_action(use_rf2o_arg)
    ld.add_action(use_rtabmap_arg)
    ld.add_action(launch_rtabmapviz_arg)
    ld.add_action(nav2_planner_arg)
    ld.add_action(nav2_controller_arg)

    # t = 0 s  — hardware drivers + state publisher
    ld.add_action(urg_node_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(rover_motor_controller_cmd)

    # t = 3 s  — rf2o needs /scan to already be publishing
    ld.add_action(TimerAction(period=3.0,  actions=[rf2o_node_cmd]))

    # t = 6 s  — RTAB-Map needs /scan + /odom (odom → base_link TF)
    ld.add_action(TimerAction(period=6.0,  actions=[rtabmap_cmd]))

    # t = 15 s — Nav2 needs the full map → odom → base_link chain
    ld.add_action(TimerAction(period=15.0, actions=[navigation_cmd]))

    return ld