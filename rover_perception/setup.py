from setuptools import setup

package_name = 'rover_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/yolov8n.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mitul',
    maintainer_email='mitul@todo.com',
    description='Object detection node using YOLOv8 for MARLIN rover',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'object_detector = rover_perception.object_detector_node:main',
        ],
    },
)