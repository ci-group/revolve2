from nca.core.genome.grammar.symbol import Symbol


class Coordinate3D:
    def __init__(self, x: int, y: int, z: int):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other):
        return Coordinate3D(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return Coordinate3D(self.x - other.x, self.y - other.y, self.z - other.z)

    def __truediv__(self, other):
        return Coordinate3D(self.x / other, self.y / other, self.z / other)

    def __repr__(self):
        return "(" + str(self.x) + " | " + str(self.y) + " | " + str(self.z) + ")"

    def __hash__(self):
        return hash((self.x, self.y, self.z))

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z


class Orientation(Symbol):
    TOP = Coordinate3D(1, 0, 0)
    RIGHT = Coordinate3D(0, 1, 0)
    DOWN = Coordinate3D(-1, 0, 0)
    LEFT = Coordinate3D(0, -1, 0)
    NEUTRAL = Coordinate3D(0, 0, 0)
    FRONT = Coordinate3D(0, 0, 1)
    BACK = Coordinate3D(0, 0, -1)

    def opposite(self):
        if self == self.TOP:
            return self.DOWN
        elif self == self.RIGHT:
            return self.LEFT
        elif self == self.DOWN:
            return self.TOP
        elif self == self.LEFT:
            return self.RIGHT
        elif self == self.NEUTRAL:
            return self.NEUTRAL
        elif self == self.FRONT:
            return self.BACK
        elif self == self.BACK:
            return self.FRONT
        raise Exception("Orientation unknown")

    @classmethod
    def directions(cls):
        return [Orientation.TOP, Orientation.RIGHT, Orientation.DOWN, Orientation.LEFT]


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
