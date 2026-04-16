from geometry_msgs.msg import Twist
import math


class DroneController:
    def __init__(self, node):
        self.node = node
        self.pub = node.create_publisher(Twist, '/cmd_vel', 10)

        self.max_vel = 0.5
        self.smooth_alpha = 0.7
        self.last_x = 0.0
        self.last_y = 0.0

    def clamp(self, v):
        return max(-self.max_vel, min(self.max_vel, v))

    def smooth(self, new, last):
        return self.smooth_alpha * last + (1 - self.smooth_alpha) * new

    def move_to_target(self, pose, target):
        msg = Twist()

        error_x = target[0] - pose[0]
        error_y = target[1] - pose[1]

        vx = self.clamp(0.6 * error_x)
        vy = self.clamp(0.6 * error_y)

        # smoothing
        vx = self.smooth(vx, self.last_x)
        vy = self.smooth(vy, self.last_y)

        self.last_x = vx
        self.last_y = vy

        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = 0.0

        self.pub.publish(msg)

    def hover(self):
        msg = Twist()
        self.pub.publish(msg)
        self.last_x = 0.0
        self.last_y = 0.0
