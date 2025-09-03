# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from launch_ros.actions import Node



def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')  
    use_rviz = LaunchConfiguration('rviz', default='false')

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('rover_bringup')
    pkg_project_gazebo = get_package_share_directory('rover_gazebo')
    pkg_project_description = get_package_share_directory('rover_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Load the SDF file from "description" package
    #sdf_file  =  os.path.join(pkg_project_description, 'models', 'diff_drive', 'model.sdf')
    #with open(sdf_file, 'r') as infp:
    #    robot_desc = infp.read()

    # Robot description
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([
            pkg_project_description,
            'models',
            'rover.urdf.xacro'
        ])]), value_type=str
    )


    # Setup to launch the simulator and Gazebo world, not paused
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [PathJoinSubstitution([
                pkg_project_gazebo,
                'worlds',
                'shapes.sdf'
            ]), ' -r']
        }.items(),
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
            'publish_frequency': 50.0,
        }]
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'rover',
            '-topic', 'robot_description',
            '-z', '0.5'
        ],
        output='screen'
    )
    

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={os.path.join(pkg_project_bringup, "config", "rover_bridge.yaml")}',
        ],
        output='screen'
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'diff_drive.rviz')],
       condition=IfCondition(use_rviz)
    )

    ld = LaunchDescription()

    ld.add_action(gz_sim)
    ld.add_action(spawn_robot)
    #ld.add_action(bridge)
    ld.add_action(robot_state_publisher)
    ld.add_action(rviz)
    return ld
