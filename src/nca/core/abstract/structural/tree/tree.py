from typing import Dict

from nca.core.abstract.structural.composite import Composite
from nca.core.abstract.sequential_identifier import NodeIdentifier
from nca.core.abstract.structural.tree.tree_helper import Coordinate3D, BiDict, Orientation, Alignment


class Tree(Composite):

    identifier = NodeIdentifier()

    def __init__(self, root_tree=None):
        super().__init__()
        self.id = self.identifier.id()
        self.root = root_tree if root_tree is not None else self

    def add(self, tree=None):
        tree = tree if tree is not None else Tree(self.root)
        self.children.append(tree)

    def graph(self) -> Dict:
        graph = {}
        explore = [(self.root, self.children)]

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


class TreeRegistry:
    def __init__(self, root_id: int):
        self.registered: BiDict = BiDict()
        self.registered[root_id] = Coordinate3D(0, 0, 0)

    def register(self, parent_id, child_id, orientation):
        new_coordinate = self.registered[parent_id] + orientation
        if new_coordinate not in self.registered.inverse:
            self.registered[child_id] = new_coordinate
        else:
            raise Exception("invalid tree")


class Tree2D(Tree):

    def __init__(self, root_tree=None):
        super().__init__(root_tree)
        self.children: Dict[Orientation, Tree2D] = {}
        self.registry = root_tree.registry if root_tree is not None else TreeRegistry(self.id)

    def add(self, orientation: Orientation, tree=None):
        if orientation in self.children.keys():
            raise Exception("already set orientation for Node2D")
        tree = tree if tree is not None else self.__class__(self.root)
        self.children[orientation] = tree
        self.registry.register(self.id, tree.id, orientation)
        return tree

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


class Tree3D(Tree2D):

    def __init__(self, root_tree=None):
        super().__init__(root_tree)
        self.children: Dict[Alignment, Tree3D] = {}
