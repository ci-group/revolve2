import unittest

from nca.core.genome.representations.tree import Node, Node2D, Orientation, Alignment, Node3D
from nca.core.genome.representations.tree_representation import TreeRepresentation, Tree2DRepresentation


class TreeRepresentationTest(unittest.TestCase):

    def test_node(self):
        root_node = Node()
        child_1_node = Node()
        child_2_node = Node()
        root_node.add(child_1_node)
        root_node.add(child_2_node)

        self.assertIsInstance(root_node, Node)
        for child in root_node.children:
            self.assertIsInstance(child, Node)

    def test_node_2d(self):
        root_node = Node2D()

        child_1_node = Node2D()
        child_2_node = Node2D()
        root_node.add(child_1_node, Alignment.LEFT)

        try:
            root_node.add(child_2_node, Alignment.LEFT)
            self.assertTrue(False)
        except Exception:
            self.assertTrue(True)

    def test_node_3d(self):
        root_node = Node3D()

        child_1_node = Node3D()
        child_2_node = Node3D()
        root_node.add(child_1_node, Alignment.FRONT)

        try:
            root_node.add(child_2_node, Alignment.FRONT)
            self.assertTrue(False)
        except Exception:
            self.assertTrue(True)
