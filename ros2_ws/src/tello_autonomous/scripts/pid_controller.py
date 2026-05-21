# Define PIDController class
# This class implements a simple proportional controller (P controller)
#
# PID stands for:
# P -> Proportional
# I -> Integral
# D -> Derivative
#
# This implementation only uses the proportional term
class PIDController:

    # Constructor function
    #
    # kp:
    # proportional gain coefficient
    #
    # Default value:
    # 0.2
    #
    # Higher kp:
    #   faster response
    #   but may cause oscillation
    #
    # Lower kp:
    #   smoother response
    #   but slower movement
    def __init__(self, kp=0.2):

        # Store proportional gain inside object
        self.kp = kp

    # Compute control output from error
    #
    # Input:
    #   error -> difference between target and current position
    #
    # Output:
    #   control command sent to drone
    #
    # Example:
    # If gate is far to the right,
    # error becomes positive,
    # controller outputs positive yaw command
    def compute(self, error):

        # Multiply error by proportional gain
        #
        # Control equation:
        #
        # output = kp × error
        #
        # int():
        # convert result into integer command
        return int(self.kp * error)
