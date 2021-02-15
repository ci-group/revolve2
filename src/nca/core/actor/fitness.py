from typing import Dict


class Fitness:

    def __init__(self, initial_value: float = 0.0):
        super().__init__()
        self.fitness = initial_value
        self.objectives: Dict[str, float] = {}

    def value(self):
        return self.fitness

    def _add(self, key, value):
        if key in self.objectives.keys():
            raise Exception("Adding duplicate fitness value")
        self.objectives[key] = value
        self._average()

    def add(self, value):
        if isinstance(value, dict):
            for key in value.keys():
                self._add(key, value[key])
        else:
            self._add("evaluation", value)

    def _average(self):
        value: float = 0.0
        for key in self.objectives.keys():
            value += self.objectives[key]
        self.fitness = value / self.objectives.__len__()
