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
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit




def generate_launch_description():
    pkg_rover_description = get_package_share_directory('rover_description')


    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_x_cmd = DeclareLaunchArgument(
        "initial_pose_x", default_value="0.0", description="Initial pose x"
    )

    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_y_cmd = DeclareLaunchArgument(
        "initial_pose_y", default_value="0.0", description="Initial pose y"
    )

    initial_pose_z = LaunchConfiguration("initial_pose_z")
    initial_pose_z_cmd = DeclareLaunchArgument(
        "initial_pose_z", default_value="0.5", description="Initial pose z"
    )

    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    initial_pose_yaw_cmd = DeclareLaunchArgument(
        "initial_pose_yaw", default_value="0.0", description="Initial pose yaw"
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time"
    )

    ### NODES ###
    spawn_entity_cmd = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "rover",
            "-topic", "robot_description",
            "-x", initial_pose_x,
            "-y", initial_pose_y,
            "-z", initial_pose_z,
            "-Y", initial_pose_yaw,
            "-allow_renaming", "true"
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    bridge_params = os.path.join(get_package_share_directory("rover_gazebo"), "config", "rover_bridge.yaml"   )
    start_gazebo_ros_bridge_cmd = Node(
        package = 'ros_gz_bridge',
        executable = 'parameter_bridge',
        arguments = [
            '--ros-args',
            '-p', f'config_file:={bridge_params}',   
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output='screen',
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rover_description"),
            "config",
            "control.yaml",
        ]
    )

    # controller_manager is provided by gz_ros_control plugin when the robot is spawned

    joint_state_broadcaster_spawner = Node(
        name="joint_state_broadcaster_spawner",
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    position_controller_spawner = Node(
        name="position_controller_spawner",
        package="controller_manager",
        executable="spawner",
        arguments=[
            "position_controller",
            '--param-file', robot_controllers,
#            "--controller-manager", "/controller_manager",
#            "--controller-manager-timeout", "120",
#            '--controller-ros-args',
#            '-r /position_controller/tf_odometry:=/tf',
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    velocity_controller_spawner = Node(
        name="velocity_controller_spawner",
        package="controller_manager",
        executable="spawner",
        arguments=[
            "velocity_controller",
            '--param-file', robot_controllers,
#            "--controller-manager", "/controller_manager",
#            "--controller-manager-timeout", "120",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    sequence_joint_state_broadcaster = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity_cmd,
                on_exit=[joint_state_broadcaster_spawner],
            )
    )

    sequence_position_controller = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[position_controller_spawner],
            )
    )

    sequence_velocity_controller = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[velocity_controller_spawner],
            )
    )

    ### LAUNCH ###
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rover_description"),
                "launch",
                "robot_state_publisher.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(initial_pose_x_cmd)
    ld.add_action(initial_pose_y_cmd)
    ld.add_action(initial_pose_z_cmd)
    ld.add_action(initial_pose_yaw_cmd)
    ld.add_action(use_sim_time_cmd)

    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_entity_cmd)
 # done via event handlers not directly.
 #   ld.add_action(joint_state_broadcaster_spawner)
 #   ld.add_action(position_controller_spawner)
 #   ld.add_action(velocity_controller_spawner)
    ld.add_action(sequence_joint_state_broadcaster)
    ld.add_action(sequence_position_controller)
    ld.add_action(sequence_velocity_controller)
    
    ld.add_action(start_gazebo_ros_bridge_cmd)  


    return ld
