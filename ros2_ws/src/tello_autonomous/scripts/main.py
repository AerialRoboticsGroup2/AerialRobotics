# Import Python time library
# Used for delays and loop timing control
import time

# Import the drone communication wrapper
# This module handles all interactions with the Tello drone
from tello_wrapper import TelloWrapper

# Import the gate detection module
# This module uses computer vision to detect gates
from gate_detector import GateDetector

# Import the navigation controller
# This module computes flight movements and PID control
from navigation import Navigation

# Import the mission manager
# This module controls the overall autonomous mission logic
from mission_manager import MissionManager


# Main program entry function
def main():

    # Create drone interface object
    # This initializes communication with the drone
    drone = TelloWrapper()

    # Connect to the Tello drone through WiFi
    drone.connect()

    # Start the drone camera video stream
    # Required for computer vision processing
    drone.start_stream()

    # Create gate detector object
    # Used to detect gates from camera images
    detector = GateDetector()

    # Create navigation controller object
    # Pass drone object so navigation can send movement commands
    navigator = Navigation(drone)

    # Create mission manager object
    # This coordinates:
    # - drone control
    # - gate detection
    # - navigation logic
    mission = MissionManager(
        drone,
        detector,
        navigator
    )

    # Boolean variable controlling the main autonomous loop
    running = True

    # Main autonomous control loop
    # Runs continuously until mission ends
    while running:

        # Execute one iteration of the mission
        # Returns:
        # True  -> continue mission
        # False -> stop mission
        running = mission.run()

        # Small delay to control loop frequency
        # 0.03 seconds ≈ 33 Hz update rate
        # Prevents CPU overload and excessive command sending
        time.sleep(0.03)


# Standard Python program entry point
# Ensures main() only runs when this file is executed directly
if __name__ == '__main__':

    # Start the autonomous drone program
    main()
