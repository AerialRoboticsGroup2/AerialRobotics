import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class DroneTest(Node):
    def __init__(self):
        super().__init__('drone_test_node')
        
        self.publisher = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )
        
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = PoseStamped()
        
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 2.0
        
        self.publisher.publish(msg)
        self.get_logger().info("Publishing test position")


def main(args=None):
    rclpy.init(args=args)
    node = DroneTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
