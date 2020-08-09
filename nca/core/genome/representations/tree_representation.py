
from nca.core.genome.representation import Representation
from nca.core.genome.representations.tree import Tree3D, Tree2D, Tree


class TreeRepresentation(Representation):

    def __init__(self, root: Tree):
        super().__init__()
        self.genome = root

    def _initialize(self):
        pass

    def compatibility(self, other) -> float:
        pass

    def visit(self, representation_visitor):
        pass


class Tree2DRepresentation(TreeRepresentation):

    def __init__(self, root: Tree2D):
        super().__init__(root)


class Tree3DRepresentation(TreeRepresentation):

    def __init__(self, root: Tree3D):
        super().__init__(root)
