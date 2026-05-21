# PID controller for autonomous drone navigation
#
# This controller is used to:
# - align the drone with gates
# - stabilize yaw motion
# - smooth trajectory tracking
# - reduce oscillation during fast flight
#
# PID formula:
#
# output =
#     kp * error
#   + ki * integral
#   + kd * derivative
#
# where:
#
# P term:
#   immediate correction
#
# I term:
#   accumulated correction
#   removes steady-state bias
#
# D term:
#   predicts motion trend
#   reduces overshoot and oscillation

import time


class PIDController:

    # Constructor function
    #
    # kp:
    #   proportional gain
    #
    # ki:
    #   integral gain
    #
    # kd:
    #   derivative gain
    #
    # output_limit:
    #   maximum absolute command output
    #
    # Example:
    #   yaw speed limit
    #
    def __init__(
        self,
        kp=0.35,
        ki=0.0005,
        kd=0.12,
        output_limit=100
    ):

        # Store PID gains
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Maximum output magnitude
        self.output_limit = output_limit

        # Integral accumulator
        self.integral = 0.0

        # Previous error value
        self.prev_error = 0.0

        # Previous timestamp
        self.prev_time = time.time()

    # Reset controller state
    #
    # Useful when:
    # - switching targets
    # - passing a gate
    # - restarting mission
    #
    def reset(self):

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    # Compute PID output
    #
    # error:
    #   target - current value
    #
    # Returns:
    #   integer drone command
    #
    def compute(self, error):

        # Get current time
        current_time = time.time()

        # Compute elapsed time
        dt = current_time - self.prev_time

        # Prevent division-by-zero
        if dt <= 0.0001:
            dt = 0.0001

        # -----------------------------
        # PROPORTIONAL TERM
        # -----------------------------
        #
        # Immediate response to error
        #
        p = self.kp * error

        # -----------------------------
        # INTEGRAL TERM
        # -----------------------------
        #
        # Accumulate historical error
        #
        self.integral += error * dt

        # Anti-windup protection
        #
        # Prevent excessive integral growth
        #
        integral_limit = 1000

        if self.integral > integral_limit:
            self.integral = integral_limit

        if self.integral < -integral_limit:
            self.integral = -integral_limit

        i = self.ki * self.integral

        # -----------------------------
        # DERIVATIVE TERM
        # -----------------------------
        #
        # Predict future trend
        #
        derivative = (error - self.prev_error) / dt

        d = self.kd * derivative

        # -----------------------------
        # FINAL PID OUTPUT
        # -----------------------------
        #
        output = p + i + d

        # Output limiting
        #
        # Prevent unsafe command magnitude
        #
        if output > self.output_limit:
            output = self.output_limit

        if output < -self.output_limit:
            output = -self.output_limit

        # Store values for next iteration
        self.prev_error = error
        self.prev_time = current_time

        # Return integer drone command
        return int(output)
