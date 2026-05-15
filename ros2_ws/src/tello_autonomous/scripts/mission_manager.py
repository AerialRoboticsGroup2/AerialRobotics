import time

from state_machine import DroneState
from config import *

class MissionManager:

    def __init__(self, drone, detector, navigator):

        self.drone = drone
        self.detector = detector
        self.navigator = navigator

        self.state = DroneState.TAKEOFF

        self.current_gate = 0

        self.start_time = time.time()

    def run(self):def run(self):

        frame = self.drone.get_frame()

        gate = self.detector.detect_gate(frame)

        elapsed = time.time() - self.start_time

        if elapsed > MISSION_TIMEOUT:
            print("Mission timeout")
            self.drone.land()
            return False

        if self.state == DroneState.TAKEOFF:

            print("Taking off")
            self.drone.takeoff()
            time.sleep(2)
            self.state = DroneState.SEARCH_GATE

        elif self.state == DroneState.SEARCH_GATE:

            if gate is None:
                self.navigator.search_gate()
            else:
                self.state = DroneState.ALIGN_GATE

        elif self.state == DroneState.ALIGN_GATE:

            if gate is None:
                self.state = DroneState.SEARCH_GATE
                return True

            aligned = self.navigator.align_gate(gate)
            if aligned:
                print(f"Gate {self.current_gate + 1} aligned")
                self.state = DroneState.PASS_GATE

        elif self.state == DroneState.PASS_GATE:

            print(f"Passing gate {self.current_gate + 1}")

            self.navigator.pass_gate()

            self.current_gate += 1

            if self.current_gate >= TOTAL_GATES:
                self.state = DroneState.LAND
            else:
                self.state = DroneState.TURN_TO_NEXT

        elif self.state == DroneState.TURN_TO_NEXT:

            print("Turning to next gate")
            self.drone.rotate(90)

            time.sleep(2)

            self.state = DroneState.SEARCH_GATE

        elif self.state == DroneState.LAND:

            print("Landing")

            self.drone.land()

            self.state = DroneState.FINISHED

        elif self.state == DroneState.FINISHED:

            return False

        return True
