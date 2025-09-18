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

    # Setup project paths
    pkg_rover_bringup = get_package_share_directory('rover_bringup')
    pkg_rover_gazebo = get_package_share_directory('rover_gazebo')
    pkg_rover_description = get_package_share_directory('rover_description')
    pkg_rover_localization = get_package_share_directory('rover_localization')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    rviz_config = os.path.join(pkg_rover_gazebo, "rviz", "default.rviz")

    ### ARGS ###
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')  
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time"
    )

    world = LaunchConfiguration("world")
    world_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(pkg_rover_gazebo, "worlds", "empty.world"),
        description="Gazebo world",
    )

    launch_gui = LaunchConfiguration("launch_gui")
    launch_gui_cmd = DeclareLaunchArgument(
        "launch_gui", default_value="True", description="Whether launch gzclient"
    )

    pause_gz = LaunchConfiguration("pause_gz")
    pause_gz_cmd = DeclareLaunchArgument(
        "pause_gz", default_value="False", description="Whether to pause gazebo"
    )

    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_rviz_cmd = DeclareLaunchArgument(
        "launch_rviz", default_value="True", description="Whether launch rviz2"
    )

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
        "initial_pose_z", default_value="0.22", description="Initial pose z"
    )

    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    initial_pose_yaw_cmd = DeclareLaunchArgument(
        "initial_pose_yaw", default_value="0.0", description="Initial pose yaw"
    )

    nav2_planner = LaunchConfiguration("nav2_planner")
    nav2_planner_cmd = DeclareLaunchArgument(
        "nav2_planner",
        default_value="SmacHybrid",
        choices=["SmacHybrid", "SmacLattice"],
        description="Nav2 planner (SmacHybrid or SmacLattice)",
    )

    nav2_controller = LaunchConfiguration("nav2_controller")
    nav2_controller_cmd = DeclareLaunchArgument(
        "nav2_controller",
        default_value="RPP",
        choices=["RPP", "TEB"],
        description="Nav2 controller (RPP or TEB)",
    )


    ### NODES ###
    rviz_cmd = Node(
       package='rviz2',
       executable='rviz2',
       arguments=["-d", rviz_config, "--ros-args", "--log-level", "Error"],
       parameters=[{"use_sim_time": use_sim_time}],
       condition=IfCondition(launch_rviz)
    )

    ### LAUNCHS ###

    # Setup to launch the simulator and Gazebo world, not paused
    # TODO - split between client and server
    gz_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [PathJoinSubstitution([
                pkg_rover_gazebo,
                'worlds',
                'shapes.sdf'
#                'obstacle_course.sdf'
            ]), ' -r -v 4']
        }.items(),
    )

    
    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_localization, "launch", "localization.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

#    navigation_cmd = IncludeLaunchDescription(
#        PythonLaunchDescriptionSource(
#            os.path.join(pkg_rover_navigation, "launch", "bringup.launch.py")
#        ),
#        launch_arguments={
#            "use_sim_time": use_sim_time,
#            "planner": nav2_planner,
#            "controller": nav2_controller,
#        }.items(),
#    )

    joy_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('teleop_twist_joy'), "launch", "teleop-launch.py")
        ),
        launch_arguments={
            "joy_config": "pdp"
        }.items(),
    )

    cmd_vel_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_gazebo, "launch/include", "cmd_vel.launch.py")
        ),
    )

    spawn_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rover_gazebo, "launch/include", "spawn.launch.py")
        ),
        launch_arguments={
            "initial_pose_x": initial_pose_x,
            "initial_pose_y": initial_pose_y,
            "initial_pose_z": initial_pose_z,
            "initial_pose_yaw": initial_pose_yaw,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(use_sim_time_cmd)
    ld.add_action(world_cmd)
    ld.add_action(launch_gui_cmd)
    ld.add_action(pause_gz_cmd)
    ld.add_action(launch_rviz_cmd)
    ld.add_action(initial_pose_x_cmd)
    ld.add_action(initial_pose_y_cmd)
    ld.add_action(initial_pose_z_cmd)
    ld.add_action(initial_pose_yaw_cmd)

# One or the other of these can be uncommented to enable Nav2 or Teleop
    #ld.add_action(nav2_planner_cmd)
    #ld.add_action(nav2_controller_cmd)
    ld.add_action(localization_cmd)
    #ld.add_action(navigation_cmd)
    ld.add_action(joy_cmd)

    ld.add_action(gz_sim_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(cmd_vel_cmd)
    ld.add_action(spawn_cmd)
    return ld
