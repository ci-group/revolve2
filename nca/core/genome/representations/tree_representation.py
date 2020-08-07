
from nca.core.abstract.composite import Composite
from nca.core.genome.representation import Representation
from nca.core.genome.representations.tree import Node3D, Node2D


class TreeRepresentation(Representation):

    def __init__(self, root: Composite):
        super().__init__()
        self.genome = root

    def _initialize(self):
        pass

    def compatibility(self, other) -> float:
        pass

    def visit(self, representation_visitor):
        pass


class Tree2DRepresentation(TreeRepresentation):

    def __init__(self, root: Node2D):
        super().__init__(root)


class Tree3DRepresentation(TreeRepresentation):

    def __init__(self, root: Node3D):
        super().__init__(root)
