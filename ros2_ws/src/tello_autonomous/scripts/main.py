import time

from tello_wrapper import TelloWrapper
from gate_detector import GateDetector
from navigation import Navigation
from mission_manager import MissionManager


def main():

    drone = TelloWrapper()

    drone.connect()
    drone.start_stream()

    detector = GateDetector()

    navigator = Navigation(drone)

    mission = MissionManager(
        drone,
        detector,
        navigator
    )

    running = True
    while running:

        running = mission.run()

        time.sleep(0.03)


if __name__ == '__main__':
    main()
