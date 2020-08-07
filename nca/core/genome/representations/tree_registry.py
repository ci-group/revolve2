from enum import Enum, auto
from typing import Dict

from nca.core.abstract.composite import Composite
from nca.core.genome.representations.tree import Node2D, Orientation


class Coordinate3D:
    def __init__(self, x: int, y: int, z: int):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, orientation: Orientation):
        return Coordinate3D(self.x + orientation.value[0], self.y + orientation.value[1], self.z + orientation.value[2])

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
            self.inverse.setdefault(value,[]).append(key)

    def __setitem__(self, key, value):
        if key in self:
            self.inverse[self[key]].remove(key)
        super(BiDict, self).__setitem__(key, value)
        self.inverse.setdefault(value,[]).append(key)

    def __delitem__(self, key):
        self.inverse.setdefault(self[key],[]).remove(key)
        if self[key] in self.inverse and not self.inverse[self[key]]:
            del self.inverse[self[key]]
        super(BiDict, self).__delitem__(key)


def tree_registry(visited, graph, node: Node2D, used):

    if node not in visited:

        visited.add(node)

        for orientation, child_node in graph[node]:

            new_coordinate = used[node.id] + orientation
            if new_coordinate not in used.inverse:
                used[child_node.id] = new_coordinate
            else:
                raise Exception("invalid tree")
            tree_registry(visited, graph, child_node, used)
