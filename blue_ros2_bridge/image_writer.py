from threading import Thread

from blue_ros2_bridge.util import BaseNode

import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import CompressedImage


class ImageWriter(BaseNode):

    def __init__(self, node):
        super().__init__(node)
        self.image_dir = self.get_parameter('image_dir', '/tmp')
        self.node.create_subscription(CompressedImage, 'image/compressed',
                                      self.on_image, 10)

    def on_image(self, msg):
        img_time = self.time_from_stamp(msg.header.stamp)
        raw = msg.data
        filename = f'{self.image_dir}/img_{img_time}.png'
        with open(filename, 'wb') as outfile:
            outfile.write(raw)
        self.log_info(f'Wrote image to {filename}')


def main():
    rclpy.init()
    node = ImageWriter(Node('image_writer'))
    rclpy.spin(node.get_node())


if __name__ == '__main__':
    main()
