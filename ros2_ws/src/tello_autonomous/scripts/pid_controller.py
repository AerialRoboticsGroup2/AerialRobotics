class PIDController:

    def __init__(self, kp=0.2):
        self.kp = kp

    def compute(self, error):
        return int(self.kp * error)
