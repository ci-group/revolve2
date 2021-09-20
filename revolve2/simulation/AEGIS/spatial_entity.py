import math
from abc import abstractmethod
from enum import Enum

import numpy as np

class Coordinate:
    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y

    def __hash__(self):
        return hash(str(self.__repr__()))

    def __add__(self, other):
        return Coordinate(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return math.sqrt(abs(self.x - other.x)**2 + abs(self.y - other.y)**2)

    def __mul__(self, multiplication):
        return Coordinate(self.x * multiplication, self.y * multiplication)

    def check(self, max_x, max_y):
        # Check whether the coordinate is within the limits.
        return (self.x < max_x and self.x >= 0.0) and (self.y < max_y and self.y >= 0.0)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __repr__(self):
        return "(%d, %d)" % (self.x, self.y)

    @classmethod
    def random_coordinate(cls, height, width):
        x, y = np.random.randint([height, width])
        return cls(x, y)

    def neighbours(self):
        coordinates = []

        for direction in Direction:
            new_coordinate = self.__add__(direction.value)
            coordinates.append(new_coordinate)

        return coordinates


class Direction(Enum):
    Top = Coordinate(1, 0)
    Right = Coordinate(0, 1)
    Bottom = Coordinate(-1, 0)
    Left = Coordinate(0, -1)


class SpatialEntity:

    def __init__(self, coordinate: Coordinate, birth_energy: float = 1.0):
        self.coordinate: Coordinate = coordinate
        self.energy: float = birth_energy

    @abstractmethod
    def step(self):
        pass

    @abstractmethod
    def render(self):
        pass

    def life(self):
        return self.energy
