import time

from config import *
from pid_controller import PIDController


class Navigation:

    def __init__(self, drone):

        # Store drone interface
        self.drone = drone

        # PID controller for yaw alignment
        # Controls left/right rotation
        self.pid_yaw = PIDController(
            kp=KP_YAW,
            ki=0.0005,
            kd=0.12,
            output_limit=60
        )

        # PID controller for altitude alignment
        self.pid_updown = PIDController(
            kp=KP_UPDOWN,
            ki=0.0003,
            kd=0.08,
            output_limit=40
        )

        # PID controller for forward speed control
        # Uses gate area as distance estimate
        self.pid_forward = PIDController(
            kp=0.015,
            ki=0.0,
            kd=0.002,
            output_limit=50
        )

        # Search state
        self.search_direction = 1

        # Timestamp for search pattern switching
        self.last_search_switch = time.time()

    # -------------------------------------------------
    # Align drone with gate while moving forward
    #
    # Returns:
    #   True  -> gate ready for pass-through
    #   False -> continue alignment
    # -------------------------------------------------
    def align_gate(self, gate):

        # Compute image-center errors
        error_x = gate['x'] - CENTER_X
        error_y = gate['y'] - CENTER_Y

        # Gate size used for distance estimation
        gate_area = gate['area']

        # ---------------------------------------------
        # PID outputs
        # ---------------------------------------------

        # Yaw correction
        yaw = self.pid_yaw.compute(error_x)

        # Vertical correction
        updown = -self.pid_updown.compute(error_y)

        # ---------------------------------------------
        # Dynamic forward speed
        #
        # Smaller area:
        # gate farther away
        #
        # Larger area:
        # gate closer
        # ---------------------------------------------

        distance_error = TARGET_GATE_AREA - gate_area

        forward_speed = self.pid_forward.compute(distance_error)

        # Ensure minimum forward motion
        if forward_speed < MIN_FORWARD_SPEED:
            forward_speed = MIN_FORWARD_SPEED

        # Ensure maximum speed safety
        if forward_speed > MAX_FORWARD_SPEED:
            forward_speed = MAX_FORWARD_SPEED

        # ---------------------------------------------
        # Alignment condition
        # ---------------------------------------------

        aligned_x = abs(error_x) < X_THRESHOLD
        aligned_y = abs(error_y) < Y_THRESHOLD

        # Gate close enough for passing
        close_enough = gate_area > PASS_GATE_AREA

        # ---------------------------------------------
        # Continuous forward flight
        #
        # Grade 6 systems do NOT stop while aligning
        # ---------------------------------------------

        self.drone.send_control(
            0,                  # left/right
            forward_speed,      # forward
            updown,             # vertical
            yaw                 # yaw rotation
        )

        # Ready to pass gate
        if aligned_x and aligned_y and close_enough:

            # Reset PID integrators
            self.pid_yaw.reset()
            self.pid_updown.reset()

            return True

        return False

    # -------------------------------------------------
    # Intelligent gate search
    #
    # Uses alternating yaw search pattern
    # -------------------------------------------------
    def search_gate(self):

        current_time = time.time()

        # Change search direction periodically
        if current_time - self.last_search_switch > 2.0:

            self.search_direction *= -1

            self.last_search_switch = current_time

        # Rotate while slowly moving upward
        self.drone.send_control(
            0,
            0,
            10,
            SEARCH_YAW_SPEED * self.search_direction
        )

    # -------------------------------------------------
    # Pass through gate aggressively
    #
    # Non-blocking forward burst
    # -------------------------------------------------
    def pass_gate(self):

        start_time = time.time()

        while time.time() - start_time < PASS_DURATION:

            # Continue flying forward
            self.drone.send_control(
                0,
                PASS_FORWARD_SPEED,
                0,
                0
            )

            # High-frequency control updates
            time.sleep(0.03)

        # Stop briefly after passing gate
        self.drone.send_control(0, 0, 0, 0)

    # -------------------------------------------------
    # Smooth turning maneuver
    #
    # Required for circular gate arrangement
    # -------------------------------------------------
    def turn_to_next_gate(self):

        start_time = time.time()

        while time.time() - start_time < TURN_DURATION:

            # Forward arc turning
            self.drone.send_control(
                0,
                TURN_FORWARD_SPEED,
                0,
                TURN_YAW_SPEED
            )

            time.sleep(0.03)

        # Stabilize after turn
        self.drone.send_control(0, 0, 0, 0)

    # -------------------------------------------------
    # Emergency stop
    # -------------------------------------------------
    def stop(self):

        self.drone.send_control(0, 0, 0, 0)
