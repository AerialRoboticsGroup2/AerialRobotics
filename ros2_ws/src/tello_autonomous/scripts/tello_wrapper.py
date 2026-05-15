from djitellopy import Tello

class TelloWrapper:

    def __init__(self):
        self.tello = Tello()

    def connect(self):
        self.tello.connect()
        print("Battery:", self.tello.get_battery())

    def start_stream(self):
        self.tello.streamon()

    def get_frame(self):
        return self.tello.get_frame_read().frame

    def takeoff(self):
        self.tello.takeoff()

    def land(self):
        self.tello.land()

    def send_control(self, lr, fb, ud, yaw):
        self.tello.send_rc_control(lr, fb, ud, yaw)

    def rotate(self, angle):
        self.tello.rotate_clockwise(angle)

    def move_forward(self, distance):
        self.tello.move_forward(distance)
