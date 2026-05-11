from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('mpu6500_imu')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'mpu6500_params.yaml']),
        description='Path to the ROS2 parameters file',
    )

    imu_node = Node(
        package='mpu6500_imu',
        executable='mpu6500_imu_node',
        name='mpu6500_imu_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            # Remap if needed, e.g. to match your robot's topic convention:
            # ('~/imu/raw',  '/imu/data_raw'),
            # ('~/imu/temp', '/imu/temperature'),
        ],
    )

    return LaunchDescription([
        params_file_arg,
        imu_node,
    ])