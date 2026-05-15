import time

from config import *
from pid_controller import PIDController

class Navigation:

    def __init__(self, drone):

        self.drone = drone

        self.pid_yaw = PIDController(KP_YAW)
        self.pid_updown = PIDController(KP_UPDOWN)

    def align_gate(self, gate):

        error_x = gate['x'] - CENTER_X
        error_y = gate['y'] - CENTER_Y
        yaw = self.pid_yaw.compute(error_x)
        updown = -self.pid_updown.compute(error_y)

        aligned_x = abs(error_x) < X_THRESHOLD
        aligned_y = abs(error_y) < Y_THRESHOLD

        if aligned_x and aligned_y:
            self.drone.send_control(0, 0, 0, 0)
            return True

        self.drone.send_control(
            0,
            0,
            updown,
            yaw
        )

        return False
    def search_gate(self):

        self.drone.send_control(
            0,
            0,
            0,
            SEARCH_YAW_SPEED
        )

    def pass_gate(self):

        self.drone.send_control(
            0,
            FORWARD_SPEED,
            0,
            0
        )

        time.sleep(2.0)

        self.drone.send_control(0, 0, 0, 0)
