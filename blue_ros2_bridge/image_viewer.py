from threading import Thread

from blue_ros2_bridge.util import BaseNode

import cv2 as cv

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage


class ImageViewer(BaseNode):

    def __init__(self, node):
        super().__init__(node)
        self.frame = None

        self.node.create_subscription(CompressedImage, 'image/compressed',
                                      self.on_image, 10)

    def on_image(self, msg):
        raw = np.frombuffer(msg.data, np.uint8)
        self.frame = cv.imdecode(raw, cv.IMREAD_UNCHANGED)
        cv.imshow('image', self.frame)
        cv.waitKey(1)


def main():
    rclpy.init()
    node = ImageViewer(Node('image_viewer'))
    rclpy.spin(node.get_node())


if __name__ == '__main__':
    main()
