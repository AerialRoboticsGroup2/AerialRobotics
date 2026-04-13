import rclpy
from rclpy.node import Node

from .controller import DroneController
from .vision import VisionSystem
from .slam_interface import SLAMInterface


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        self.controller = DroneController(self)
        self.vision = VisionSystem(self)
        self.slam = SLAMInterface(self)

        # boundary of security (vitual wall)
        self.safe_x = 2.0
        self.safe_y = 2.0
        self.danger_margin = 0.3

        self.timer = self.create_timer(0.1, self.loop)

    def is_safe(self, pose):
        x, y = pose

        if abs(x) > self.safe_x - self.danger_margin:
            return False
        if abs(y) > self.safe_y - self.danger_margin:
            return False
        return True

    def emergency_behavior(self):
        self.get_logger().warn("Near boundary! Emergency hover")
        self.controller.hover()

    def loop(self):
        pose = self.slam.get_pose()
        target = self.vision.detect_ring()

        # security measure
        if not self.is_safe(pose):
            self.emergency_behavior()
            return

        if target is not None:
            self.controller.move_to_target(pose, target)
        else:
            self.controller.hover()


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
