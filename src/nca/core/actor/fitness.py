from typing import Dict


class Fitness(Dict[str, float]):

    master_key = 'fitness'

    def __init__(self, initial_value: float = 0.0):
        super().__init__()
        self[self.master_key] = initial_value
        self.clean = True  # changes to fitness dictionary

    def value(self):
        if not self.clean:
            self._average()
        return self[self.master_key]

    def add(self, key: str, value: float):
        if key in self.keys():
            raise Exception("Adding duplicate fitness value")
        self[key] = value
        self.clean = False

    def _average(self):
        sum: float = 0
        for key in self.keys():
            if key == self.master_key:
                continue
            sum += self[key]
        self[self.master_key] = sum / (self.__len__() - 1)  # Exclude master key from length calculation
        self.clean = True
