import copy

import numpy as np

from abstract.structural.tree.tree_helper import Orientation
from nca.core.evolution.conditions.initialization import Initialization
from nca.core.genome.representations.representation import Representation
from abstract.structural.tree.tree import Tree, CoordinateTree


class TreeRepresentation(Representation):

    def __init__(self, root: Tree, initialization: Initialization = None):
        super().__init__(initialization=initialization)
        self.append(root)

    def _initialize(self):
        pass

    def compatibility(self, other) -> float:
        pass

    def visit(self, representation_visitor):
        pass


class CoordinateTreeRepresentation(TreeRepresentation):

    def __init__(self, root: CoordinateTree):
        super().__init__(root)

    def root(self):
        return self[0]

    def random_element(self) -> (Tree, Tree, Orientation):
        parent = self.root().get_random_parent()
        orientation = np.random.choice(list(parent.children.keys()))
        child = parent.children[orientation]
        return parent, child, orientation

    def enforce_symmetry(self):
        root: CoordinateTree = self.root()
        repeated_child_top = copy.deepcopy(root.children[Orientation.TOP])
        root.connect(repeated_child_top, Orientation.DOWN)
        repeated_child_right = copy.deepcopy(root.children[Orientation.RIGHT])
        root.connect(repeated_child_right, Orientation.LEFT)
