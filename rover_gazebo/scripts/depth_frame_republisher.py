#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from copy import deepcopy


class DepthFrameRepublisher(Node):
    def __init__(self):
        super().__init__('depth_frame_republisher')

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera_rgbd/depth_image',
            self.depth_callback,
            10
        )

        self.publisher = self.create_publisher(
            Image,
            '/camera/depth_optical/image_raw',
            10
        )

        self.get_logger().info('Depth frame republisher started')

    def depth_callback(self, msg):
        # Create a copy of the message with the new frame_id
        new_msg = deepcopy(msg)
        new_msg.header.frame_id = 'camera_link_optical'

        # Republish with the new frame
        self.publisher.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthFrameRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()