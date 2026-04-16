import random


class SLAMInterface:
    def __init__(self, node):
        self.node = node
        self.x = 0.0
        self.y = 0.0

    def get_pose(self):
        # immitate the noise of SLAM
        self.x += random.uniform(-0.05, 0.05)
        self.y += random.uniform(-0.05, 0.05)

        return (self.x, self.y)
