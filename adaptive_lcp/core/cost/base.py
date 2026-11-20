"""方向惩罚模型。第一版返回 1.0，第二版接入坡向数组。"""


class DirectionModel:
    def __init__(self, k_up: float = 0.0, k_down: float = 0.0):
        self.k_up = k_up
        self.k_down = k_down

    def multiplier(self, theta: float, rows, cols):
        return 1.0

    def is_identity(self) -> bool:
        return self.k_up == 0.0 and self.k_down == 0.0
