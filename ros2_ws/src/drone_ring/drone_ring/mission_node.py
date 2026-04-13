import rclpy
from rclpy.node import Node

from .controller import DroneController
from .vision import VisionSystem
from .slam_interface import SLAMInterface


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        self.get_logger().info("Mission Node Started")

        self.controller = DroneController(self)
        self.vision = VisionSystem(self)
        self.slam = SLAMInterface(self)

        self.timer = self.create_timer(0.1, self.loop)

    def loop(self):
        # 获取定位
        pose = self.slam.get_pose()

        # 获取视觉目标
        target = self.vision.detect_ring()

        if target is not None:
            self.get_logger().info(f"Target detected: {target}")

            # 控制飞行
            self.controller.move_to_target(pose, target)
        else:
            self.get_logger().info("Searching target...")
            self.controller.hover()


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()import rclpy
from rclpy.node import Node

from .controller import DroneController
from .vision import VisionSystem
from .slam_interface import SLAMInterface


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        self.get_logger().info("Mission Node Started")

        self.controller = DroneController(self)
        self.vision = VisionSystem(self)
        self.slam = SLAMInterface(self)

        self.timer = self.create_timer(0.1, self.loop)

    def loop(self):
        # 获取定位
        pose = self.slam.get_pose()

        # 获取视觉目标
        target = self.vision.detect_ring()

        if target is not None:
            self.get_logger().info(f"Target detected: {target}")

            # 控制飞行
            self.controller.move_to_target(pose, target)
        else:
            self.get_logger().info("Searching target...")
            self.controller.hover()


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
