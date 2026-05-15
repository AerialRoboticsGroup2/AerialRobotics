from enum import Enum

class DroneState(Enum):

    TAKEOFF = 0
    SEARCH_GATE = 1
    ALIGN_GATE = 2
    PASS_GATE = 3
    TURN_TO_NEXT = 4
    LAND = 5
    FINISHED = 6
