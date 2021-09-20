import numpy as np

from revolve2.nca.core.evolution.conditions.initialization import Initialization, ValuedInitialization


class CategoricalInitialization(Initialization):
    def __init__(self, number_of_elements: int, start: int = 1):
        super().__init__()
        self.number_of_elements: int = number_of_elements
        self.start = start

    def __call__(self, size: int):
        return np.random.randint(self.start, self.number_of_elements, size).tolist()


class BinaryInitialization(ValuedInitialization):

    def __call__(self, length: int):
        return np.random.randint(2, size=length).tolist()


class GrayCodingInitialization(ValuedInitialization):

    def __call__(self, length: int):
        raise Exception("Not implemented Gray Coding Initialization")


class IntegerInitialization(ValuedInitialization):

    def __call__(self, length: int):
        return np.random.randint(self.configuration.min_range, self.configuration.max_range,
                                 size=length).tolist()


class UniqueIntegerInitialization(ValuedInitialization):

    def __call__(self, length: int):
        return (np.random.choice(self.configuration.max_range - self.configuration.min_range,
                                 size=length, replace=False) + self.configuration.min_range).tolist()



class UniformInitialization(ValuedInitialization):

    def __call__(self, size: int):
        return np.random.uniform(self.configuration.min_range, self.configuration.max_range, size).tolist()


class GaussianInitialization(ValuedInitialization):

    def __call__(self, size: int):
        mean = (self.configuration.max_range + self.configuration.min_range) / 2
        scale = (self.configuration.max_range - self.configuration.min_range) / 2
        return np.random.normal(mean, scale, size).tolist()
