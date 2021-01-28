from typing import Dict, List
import numpy as np

from nca.core.abstract.structural.composite import Composite
from nca.core.abstract.sequential_identifier import NodeIdentifier
from nca.core.abstract.structural.tree.tree_helper import Coordinate3D, BiDict, Orientation


class Tree(Composite):

    identifier = NodeIdentifier()

    def __init__(self, root_tree=None):
        super().__init__()
        self.id = self.identifier.id()

    def add(self, tree=None):
        tree = tree if tree is not None else Tree()
        self.children.append(tree)

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

"""
class TreeRegistry:
    def __init__(self, root_id: int):
        self.registered: BiDict = BiDict()
        self.registered[root_id] = Coordinate3D(0, 0, 0)

    def register(self, parent_id, child_id, orientation):
        new_coordinate = self.registered[parent_id] + orientation.value
        if new_coordinate not in self.registered.inverse:
            self.registered[child_id] = new_coordinate
        else:
            raise Exception("invalid tree")
"""


class CoordinateTree(Tree):

    def __init__(self, root_tree=None):
        super().__init__(root_tree)
        self.children: Dict[Orientation, CoordinateTree] = {}
        #self.registry = root_tree.registry if root_tree is not None else TreeRegistry(self.id)

    def add(self, orientation: Orientation, tree: Tree = None):
        if orientation in self.children.keys():
            raise Exception("already set orientation for Node2D")
        tree = tree if tree is not None else CoordinateTree()
        tree.children[orientation.opposite()] = self
        self.children[orientation] = tree
        #self.registry.register(self.id, tree.id, orientation)
        return tree

    def nodes(self, include_root=False) -> List:
        nodes = []

        if include_root:
            explore = [self]
        else:
            explore = [child for child in self.children]

        for node in explore:
            nodes.append(node)
            if len(node.children.keys()) > 0:
                for child_key, child_node in node.children.items():

                    if child_node not in explore:
                        explore.append(child_node)

        return nodes

    def connect(self, new_tree, orientation: Orientation):
        self.children[orientation] = new_tree
        new_tree.children[orientation.opposite()] = self

    def disconnect(self, orientation: Orientation):
        del self.children[orientation]

    def get_random_parent(self):
        # Choose any child node, not the root node
        return np.random.choice(self.nodes(include_root=True))

    def get_random_child(self):
        # Choose any child node, not the root node
        return np.random.choice(self.nodes(include_root=False))
