import random


class PerformanceMeasures:

    def __init__(self):
        self.velocity: float        = 0.0
        self.displacement: float    = 0.0
        self.rotation: float        = 0.0
        self.balance: float         = 0.0

    def test(self):
        self.velocity       = random.uniform(0.0, 1.0)
        self.displacement   = random.uniform(0.0, 1.0)
        self.rotation       = random.uniform(0.0, 1.0)
        self.balance        = random.uniform(0.0, 1.0)
