from enum import Enum

from nca.core.genome.grammar.grammar import Symbol


class Orientation(Symbol):
    TOP = (1, 0, 0)
    RIGHT = (0, 1, 0)
    DOWN = (-1, 0, 0)
    LEFT = (0, -1, 0)


class Alignment(Orientation, Enum):
    FRONT = (0, 0, 1)
    BACK = (0, 0, -1)


class Coordinate3D:
    def __init__(self, x: int, y: int, z: int):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other):
        if isinstance(other, Orientation):
            return Coordinate3D(self.x + other.value[0], self.y + other.value[1], self.z + other.value[2])
        else:
            return Coordinate3D(self.x + other.x, self.y + other.y, self.z + other.z)

    def __repr__(self):
        return "(" + str(self.x) + " | " + str(self.y) + " | " + str(self.z) + ")"

    def __hash__(self):
        return hash((self.x, self.y, self.z))

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z


class BiDict(dict):
    def __init__(self, *args, **kwargs):
        super(BiDict, self).__init__(*args, **kwargs)
        self.inverse = {}
        for key, value in self.items():
            self.inverse.setdefault(value, []).append(key)

    def __setitem__(self, key, value):
        if key in self:
            self.inverse[self[key]].remove(key)
        super(BiDict, self).__setitem__(key, value)
        self.inverse.setdefault(value, []).append(key)

    def __delitem__(self, key):
        self.inverse.setdefault(self[key], []).remove(key)
        if self[key] in self.inverse and not self.inverse[self[key]]:
            del self.inverse[self[key]]
        super(BiDict, self).__delitem__(key)
