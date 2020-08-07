import unittest

from nca.core.genome.representations.tree import Node, Node2D, Orientation, Alignment, Node3D
from nca.core.genome.representations.tree_representation import TreeRepresentation, Tree2DRepresentation


class TreeRepresentationTest(unittest.TestCase):

    def test_tree(self):
        root_node = Node()
        child_1_node = Node()
        child_2_node = Node()
        root_node.add(child_1_node)
        root_node.add(child_2_node)

        representation = TreeRepresentation(root_node)

        self.assertIsInstance(representation, TreeRepresentation)
        self.assertIsInstance(representation.genome, Node)

    def test_tree_2d(self):
        root_node = Node2D()

        representation = Tree2DRepresentation(root_node)

        child_1_node = Node2D()
        child_2_node = Node2D()
        root_node.add(child_1_node, Orientation.TOP)
        root_node.add(child_2_node, Orientation.DOWN)

        self.assertIsInstance(representation.genome, Node2D)
        for key, element in representation.genome.children.items():
            self.assertIsInstance(element, Node2D)

    def test_tree_3d(self):
        root_node = Node3D()

        representation = Tree2DRepresentation(root_node)

        child_1_node = Node3D()
        child_2_node = Node3D()
        root_node.add(child_1_node, Alignment.TOP)
        root_node.add(child_2_node, Alignment.BACK)

        self.assertIsInstance(representation.genome, Node3D)
        for key, element in representation.genome.children.items():
            self.assertIsInstance(element, Node3D)
