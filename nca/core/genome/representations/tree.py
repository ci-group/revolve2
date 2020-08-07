from enum import Enum, auto
from typing import Dict

from nca.core.abstract.composite import Composite
from nca.core.abstract.sequential_identifier import NodeIdentifier


class Node(Composite):
    identifier = NodeIdentifier()

    def __init__(self):
        super().__init__()
        self.id = self.identifier.id()

    def graph(self) -> Dict:
        graph = {}
        explore = [(self, self.children)]

        for node, children in explore:
            for element in children:
                if node not in graph.keys():
                    graph[node] = [element]
                else:
                    graph[node].append(element)

                if len(element.children) > 0:
                    explore.append((element, element.children))
                else:
                    graph[element] = []

        return graph


class Orientation(Enum):
    TOP = (1, 0, 0)
    RIGHT = (0, 1, 0)
    DOWN = (-1, 0, 0)
    LEFT = (0, -1, 0)


class Node2D(Node):

    def __init__(self):
        super().__init__()
        self.children: Dict[Orientation, Node2D] = {}

    def add(self, node, orientation: Orientation):
        if orientation in self.children.keys():
            raise Exception("already set orientation for Node2D")

        self.children[orientation] = node

    def graph(self) -> Dict:
        graph = {}
        explore = [(self, self.children)]

        for node, children in explore:
            for key, value in children.items():
                if node not in graph.keys():
                    graph[node] = [(key, value)]
                else:
                    graph[node].append((key, value))

                if len(value.children) > 0:
                    explore.append((value, value.children))
                else:
                    graph[value] = []

        return graph

class Alignment(Orientation, Enum):
    FRONT = (0, 0, 1)
    BACK = (0, 0, -1)


class Node3D(Node2D):

    def __init__(self):
        super().__init__()
        self.children: Dict[Alignment, Node3D] = {}
