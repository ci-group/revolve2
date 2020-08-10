import string
from typing import Dict

import numpy as np


class SimulationMeasures:

    def __init__(self):
        self.measures: Dict[string] = {'distances': [], 'times': [], 'batteries': [], 'positions': []}

    def test(self):
        for key in self.measures.keys():
            self.measures[key] = np.random.uniform(0.0, 1.0, 10)
