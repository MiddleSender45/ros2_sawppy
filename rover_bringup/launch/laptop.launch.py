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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch import conditions
from launch.conditions import IfCondition


def generate_launch_description():
    rover_bringup_shared_dir = get_package_share_directory("rover_bringup")
    #rover_motor_controller_shared_dir = get_package_share_directory(
    #    "rover_motor_controller_cpp"
    #)
    #pkg_path = get_package_share_directory("rover_gazebo")
    #rover_teleop_shared_dir = get_package_share_directory("rover_teleop")
    #pkg_rover_localization = get_package_share_directory("rover_localization")
    #pkg_rover_navigation = get_package_share_directory("rover_navigation")

    #rviz_config = os.path.join(pkg_path, "rviz", "default.rviz")

    #env1 = SetEnvironmentVariable("RCUTILS_LOGGING_USE_STDOUT", "1")
    #env2 = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    # Robot State Publisher
    #use_robot_state_publisher_arg = DeclareLaunchArgument(
    #    "use_robot_state_publisher",
    #    default_value="true",
    #    description="Whether to launch the robot_state_publisher node",
    #)
    #use_robot_state_publisher = LaunchConfiguration("use_robot_state_publisher")

    #robot_description_pkg_share = get_package_share_directory("rover_description")

    #robot_state_publisher_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(robot_description_pkg_share, "launch", "robot_state_publisher.launch.py")
    #    ),
    #    condition=conditions.IfCondition(use_robot_state_publisher),
    #)
    # Navigation Stack
    #nav2_planner = LaunchConfiguration("nav2_planner")
    #nav2_planner_cmd = DeclareLaunchArgument(
    #    "nav2_planner",
    #    default_value="SmacHybrid",
    #    choices=["SmacHybrid", "SmacLattice"],
    #    description="Nav2 planner (SmacHybrid or SmacLattice)",
    #)

    #nav2_controller = LaunchConfiguration("nav2_controller")
    #nav2_controller_cmd = DeclareLaunchArgument(
    #    "nav2_controller",
    #    default_value="RPP",
    #    choices=["RPP", "TEB"],
    #    description="Nav2 controller (RPP or TEB)",
    #)

    # Localization and Navigation
    #localization_cmd = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(pkg_rover_localization, "launch", "localization.launch.py")
    #    ),
    #   launch_arguments={"use_sim_time": "False"}.items(),
    #)

    #navigation_cmd = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(pkg_rover_navigation, "launch", "bringup.launch.py")
    #    ),
    #    launch_arguments={
    #        "use_sim_time": "False",
    #        "planner": nav2_planner,
    #        "controller": nav2_controller,
    #    }.items(),
    #)

    # Rviz launch
    #launch_rviz = LaunchConfiguration("launch_rviz")
    #launch_rviz_cmd = DeclareLaunchArgument(
    #    "launch_rviz", default_value="True", description="Whether launch rviz2"
    #)

    #rviz_cmd = Node(
    #    name="rviz",
    #    package="rviz2",
    #    executable="rviz2",
    #    arguments=["-d", rviz_config, "--ros-args", "--log-level", "Error"],
    #    parameters=[{"use_sim_time": False}],
    #    condition=IfCondition(PythonExpression([launch_rviz])),
    #    remappings=[
    #        ('camera/image_raw', '/kinect/image_raw'),
    #        ('camera/depth/image_raw', '/kinect/depth/image_raw'),
    #        ('camera/camera_info', '/kinect/camera_info'),
    #    ],
    #)

    # LIDAR
    use_lidar_arg = DeclareLaunchArgument(
        "use_lidar",
        default_value="true",
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
    #use_kinect_arg = DeclareLaunchArgument(
    #    "use_kinect",
    #    default_value="true",
    #    description="Whether to launch the Kinect node",
    #)
    #use_kinect = LaunchConfiguration("use_kinect")

    #kinect_pkg_share = get_package_share_directory("kinect_ros2")
    #kinect_node_action_cmd = Node(
    #    package="kinect_ros2",
    #    executable="kinect_ros2_node",
    #    namespace="kinect",
    #    condition=conditions.IfCondition(use_kinect),
    #    #additional_env={
        #    "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        #    "ROS_DISCOVERY_SERVER": ros_discovery_server_addr,
        #},
    #)

    #teleop_twist_joy_action_cmd = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(rover_teleop_shared_dir, "launch", "joy_teleop.launch.py")
    #    ),
    #)

    # motor controller node(talks to servo board)
    #rover_motor_controller_action_cmd = IncludeLaunchDescription(
    #  PythonLaunchDescriptionSource(
    #        os.path.join(
    #           rover_motor_controller_shared_dir, "launch", "motor_controller.launch.py"
    #        )
    #    ),
    #)

    # Perception (YOLO) - commented out for troubleshooting
    # rover_perception_shared_dir = get_package_share_directory("rover_perception")
    # object_detector_node_cmd = Node(
    #     package="rover_perception",
    #     executable="object_detector",
    #     name="object_detector",
    #     output="screen",
    # )

    ld = LaunchDescription()

    #ld.add_action(env1)
    #ld.add_action(env2)

    ld.add_action(use_lidar_arg)
    #ld.add_action(use_kinect_arg)
    #ld.add_action(use_robot_state_publisher_arg)
    #ld.add_action(nav2_planner_cmd)
    #ld.add_action(nav2_controller_cmd)
    #ld.add_action(launch_rviz_cmd)

    #ld.add_action(urg_node_action_cmd)
    #ld.add_action(kinect_node_action_cmd)
    # ld.add_action(teleop_twist_joy_action_cmd)
    #ld.add_action(rover_motor_controller_action_cmd)
    #ld.add_action(robot_state_publisher_launch)
    # Nav and Localization launch
    #ld.add_action(TimerAction(period=5.0, actions=[localization_cmd]))
    #ld.add_action(TimerAction(period=10.0, actions=[navigation_cmd]))
    #ld.add_action(TimerAction(period=12.0, actions=[rviz_cmd]))
    # ld.add_action(object_detector_node_cmd)

    return ld