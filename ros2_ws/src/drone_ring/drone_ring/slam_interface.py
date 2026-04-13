import random


class SLAMInterface:
    def __init__(self, node):
        self.node = node

    def get_pose(self):
        # 模拟无人机位置 (x, y)
        return (random.uniform(-2, 2), random.uniform(-2, 2))
