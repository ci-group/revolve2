import numpy as np

from nca.core.evolution.conditions.initialization import Initialization


class CategoricalInitialization(Initialization):
    def __init__(self, number_of_elements: int):
        super().__init__()
        self.number_of_elements: int = number_of_elements

    def algorithm(self, size: int):
        return np.random.randint(1, self.number_of_elements, size)


class UniformInitialization(Initialization):

    def __init__(self):
        super().__init__()

    def algorithm(self, size: int):
        return np.random.uniform(self.configuration.min_range, self.configuration.max_range, size)


class GaussianInitialization(Initialization):

    def __init__(self):
        super().__init__()

    def algorithm(self, size: int):
        mean = (self.configuration.max_range + self.configuration.min_range) / 2
        scale = (self.configuration.max_range - self.configuration.min_range) / 2
        return np.random.normal(mean, scale, size)
