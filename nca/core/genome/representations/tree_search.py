from enum import Enum, auto
from typing import Dict

from nca.core.abstract.composite import Composite
from nca.core.genome.representations.tree import Node2D, Orientation


def depth_first_search(visited, graph, node):
    if node not in visited:
        visited.add(node)
        for neighbour, element in graph[node]:
            depth_first_search(visited, graph, element)


root = Node2D()
root.add(Node2D(), Orientation.TOP)

child = Node2D()
child.add(Node2D(), Orientation.LEFT)
child.add(Node2D(), Orientation.RIGHT)
child.add(Node2D(), Orientation.DOWN)
root.add(child, Orientation.DOWN)
print(root.graph().keys())
depth_first_search(set(), root.graph(), root)
