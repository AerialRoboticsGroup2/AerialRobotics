from geometry_msgs.msg import Twist


class DroneController:
    def __init__(self, node):
        self.node = node
        self.publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    def move_to_target(self, pose, target):
        msg = Twist()

        # 简单P控制（示例）
        error_x = target[0] - pose[0]
        error_y = target[1] - pose[1]

        msg.linear.x = 0.5 * error_x
        msg.linear.y = 0.5 * error_y
        msg.linear.z = 0.0

        self.publisher.publish(msg)
        self.node.get_logger().info("Moving to target")

    def hover(self):
        msg = Twist()
        self.publisher.publish(msg)
