from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mpu6500_imu'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 Jazzy driver for the MPU6500 IMU over I2C',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mpu6500_imu_node = mpu6500_imu.mpu6500_imu_node:main',
        ],
    },
)