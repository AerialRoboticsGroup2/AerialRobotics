import random


class VisionSystem:
    def __init__(self, node):
        self.node = node

    def detect_ring(self):
        # stablized detection
        if random.random() > 0.4:
            return (
                random.uniform(-0.8, 0.8),
                random.uniform(-0.8, 0.8)
            )
        return None
