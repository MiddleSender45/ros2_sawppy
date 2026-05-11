from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("rover_motion")

    return LaunchDescription([
        # ── Launch arguments (override on the CLI) ───────────────────────────
        DeclareLaunchArgument("i2c_bus",      default_value="/dev/i2c-1"),
        DeclareLaunchArgument("i2c_addr",     default_value="104"),   # 0x68 decimal
        DeclareLaunchArgument("frame_id",     default_value="imu_link"),
        DeclareLaunchArgument("frequency",    default_value="50.0"),
        DeclareLaunchArgument("publish_temp", default_value="true"),

        # ── IMU node ─────────────────────────────────────────────────────────
        Node(
            package="rover_motion",
            executable="imu_node",
            name="imu_node",
            output="screen",
            parameters=[
                os.path.join(pkg_share, "config", "imu_params.yaml"),
                {
                    "i2c_bus":      LaunchConfiguration("i2c_bus"),
                    "i2c_addr":     LaunchConfiguration("i2c_addr"),
                    "frame_id":     LaunchConfiguration("frame_id"),
                    "frequency":    LaunchConfiguration("frequency"),
                    "publish_temp": LaunchConfiguration("publish_temp"),
                },
            ],
        ),
    ])
