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


from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    parameters = [
        {
            "frame_id": "base_link",
            # "guess_frame_id": "odom",
            "guess_frame_id": "",
            "subscribe_depth": True,
            "subscribe_rgb": True,
            # Dealing with slow gazebo inputs
            "approx_sync": True,
            "approx_sync_max_interval": 2.0,
            "sync_queue_size": 10,
            "wait_for_transform_duration": 0.5,
            "publish_tf": False,
            "wait_imu_to_init": False,
            "publish_null_when_lost": False,
            # 0=TORO, 1=g2o, 2=GTSAM and 3=Ceres
            "Optimizer/Strategy": "2",
            "Optimizer/GravitySigma": "0.0",
            # 0=Frame-to-Map (F2M) 1=Frame-to-Frame (F2F) 2=Fovis 3=viso2 4=DVO-SLAM 5=ORB_SLAM2 6=OKVIS 7=LOAM 8=MSCKF_VIO 9=VINS-Fusion 10=OpenVINS 11=FLOAM 12=Open3D
            "Odom/Strategy": "0",
            "Odom/ResetCountdown": "1",
            "Odom/Holonomic": "false",
            # 0=No filtering 1=Kalman filtering 2=Particle filtering
            "Odom/FilteringStrategy": "1",
            "Odom/ParticleSize": "500",
            "Odom/GuessMotion": "true",
            "Odom/AlignWithGround": "false",
            "Odom/ImageDecimation": "2",
            "Odom/MaxFeatures": "800",
            "OdomF2M/MaxSize": "500",
            "OdomF2M/ScanMaxSize": "2000",
            "GFTT/MinDistance": "10.0",
            "GFTT/QualityLevel": "0.01",
            "GFTT/BlockSize": "3",
            "GFTT/UseHarrisDetector": "true",
            "GFTT/K": "0.04",
            "SURF/Extended": "true",
            "SURF/HessianThreshold": "1000",
            "SURF/Octaves": "2",
            "SURF/OctaveLayers": "2",
            "SURF/Upright": "false",
            "SURF/GpuVersion": "false",
            "SURF/GpuKeypointsRatio": "0.01",
            "SIFT/NFeatures": "200",
            "SIFT/NOctaveLayers": "2",
            "SIFT/RootSIFT": "false",
            "FREAK/OrientationNormalized": "true",
            "FREAK/ScaleNormalized": "true",
            "FREAK/PatternScale": "30",
            "FREAK/NOctaves": "4",
            "KAZE/Extended": "true",
            "KAZE/Upright": "false",
            "KAZE/NOctaves": "2",
            "KAZE/NOctaveLayers": "2",
            # 0=DIFF_PM_G1, 1=DIFF_PM_G2, 2=DIFF_WEICKERT or 3=DIFF_CHARBONNIER
            "KAZE/Diffusivity": "1",
            "BRIEF/Bytes": "64",
            # Motion estimation approach: 0:3D->3D, 1:3D->2D (PnP), 2:2D->2D (Epipolar Geometry)
            "Vis/EstimationType": "1",
            "Vis/ForwardEstOnly": "true",
            # 0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector
            "Vis/Debug": "1",
            "Vis/DrawKeypoints": "true",
            "Vis/FeatureType": "8",
            "Vis/DepthAsMask": "false",
            "Vis/CorGuessWinSize": "40",
            "Vis/MaxFeatures": "800",
            "Vis/MinDepth": "0.20",
            "Vis/MaxDepth": "5.0",
            # 0=Features Matching, 1=Optical Flow
            "Vis/CorType": "0",
            # kNNFlannNaive=0, kNNFlannKdTree=1, kNNFlannLSH=2, kNNBruteForce=3, kNNBruteForceGPU=4, BruteForceCrossCheck=5, SuperGlue=6, GMS=7
            "Vis/CorNNType": "3",
            "Vis/CorNNK": "2",
            "Vis/CorNNDr": "0.8",
        }
    ]

    remappings = [
        ("rgb/image", "camera/rgbd/image"),
        ("rgb/camera_info", "camera/rgbd/camera_info"),
        ("depth/image", "camera/rgbd/depth_image"),
        ("imu", "imu"),
        ("odom", "odom_rgbd"),
    ]

    config_dir = os.path.join(
        get_package_share_directory("rover_localization"),
        "config",
        "rgbd_odometry_qos_overrides.yaml",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    # Add use_sim_time to parameters
    parameters[0]["use_sim_time"] = ParameterValue(use_sim_time, value_type=bool)

    return LaunchDescription(
        [
            use_sim_time_arg,
            Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                output="log",
                parameters=parameters + [config_dir],
                remappings=remappings,
                arguments=["--ros-args", "--log-level", "Info"],
            ),
        ]
    )
