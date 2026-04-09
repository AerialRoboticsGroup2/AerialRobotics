#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

import time
import math


class OffboardControl(Node):

    def __init__(self):
        super().__init__('offboard_control')

        # Publishers
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        # Subscribers
        self.pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.pos_callback, 10)
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, 10)

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        # State
        self.current_position = None
        self.nav_state = None
        self.setpoint_counter = 0

        # 6 "ring" positions (x, y, z)
        self.waypoints = [
            [0.0, 0.0, -2.0],
            [2.0, 0.0, -2.0],
            [2.0, 2.0, -2.0],
            [0.0, 2.0, -2.0],
            [-2.0, 2.0, -2.0],
            [-2.0, 0.0, -2.0]
        ]

        self.current_wp = 0

    def pos_callback(self, msg):
        self.current_position = msg

    def status_callback(self, msg):
        self.nav_state = msg.nav_state

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_trajectory(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.traj_pub.publish(msg)

    def publish_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def arm(self):
        self.publish_command(400)  # VEHICLE_CMD_COMPONENT_ARM_DISARM
        self.get_logger().info("Arm command sent")

    def set_offboard_mode(self):
        self.publish_command(176, 1.0, 6.0)  # OFFBOARD mode
        self.get_logger().info("Offboard mode requested")

    def reached_waypoint(self, wp):
        if self.current_position is None:
            return False

        dx = self.current_position.x - wp[0]
        dy = self.current_position.y - wp[1]
        dz = self.current_position.z - wp[2]

        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        return dist < 0.3

    def timer_callback(self):

        self.publish_offboard_mode()

        # Send initial setpoints before switching mode
        if self.setpoint_counter < 20:
            self.publish_trajectory(0.0, 0.0, -2.0)
            self.setpoint_counter += 1
            return

        # Switch to offboard and arm
        if self.setpoint_counter == 20:
            self.set_offboard_mode()
            self.arm()
            self.setpoint_counter += 1

        # Follow waypoints
        if self.current_wp < len(self.waypoints):
            wp = self.waypoints[self.current_wp]
            self.publish_trajectory(wp[0], wp[1], wp[2])

            if self.reached_waypoint(wp):
                self.get_logger().info(f"Reached waypoint {self.current_wp}")
                self.current_wp += 1
                time.sleep(1)

        else:
            self.get_logger().info("All rings passed!")
            self.publish_trajectory(0.0, 0.0, -2.0)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
