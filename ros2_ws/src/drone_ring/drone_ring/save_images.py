#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        
        self.count = 0
        self.save_path = os.path.expanduser('~/drone_images')
        os.makedirs(self.save_path, exist_ok=True)

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        filename = f"{self.save_path}/img_{self.count:04d}.png"
        cv2.imwrite(filename, frame)
        
        self.get_logger().info(f"Saved {filename}")
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
