import unittest

from nca.core.abstract.composite.tree import Tree, Tree2D, Tree3D
from nca.core.abstract.composite.tree_helper import Orientation, Alignment
from nca.core.abstract.composite.tree_representation import TreeRepresentation, Tree2DRepresentation, \
    Tree3DRepresentation


class TreeRepresentationTest(unittest.TestCase):

    def test_tree(self):
        root_node = Tree()
        root_node.add()
        root_node.add()

        representation = TreeRepresentation(root_node)

        self.assertIsInstance(representation, TreeRepresentation)
        self.assertIsInstance(representation.genome, Tree)

    def test_tree_2d(self):
        root_node = Tree2D()

        representation = Tree2DRepresentation(root_node)

        root_node.add(Orientation.TOP)
        root_node.add(Orientation.DOWN)

        self.assertIsInstance(representation.genome, Tree2D)
        for key, element in representation.genome.children.items():
            self.assertIsInstance(element, Tree2D)

    def test_tree_3d(self):
        root_node = Tree3D()

        representation = Tree3DRepresentation(root_node)

        root_node.add(Alignment.TOP)
        root_node.add(Alignment.BACK)

        self.assertIsInstance(representation.genome, Tree3D)
        for key, element in representation.genome.children.items():
            self.assertIsInstance(element, Tree3D)
