import random


class VisionSystem:
    def __init__(self, node):
        self.node = node

    def detect_ring(self):
        # 模拟检测（50%概率看到目标）
        if random.random() > 0.5:
            # 返回目标位置 (x, y)
            return (random.uniform(-1, 1), random.uniform(-1, 1))
        else:
            return None
