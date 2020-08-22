import unittest

from nca.core.abstract.structural.tree.tree import Tree, Tree2D, Tree3D
from nca.core.abstract.structural.tree.tree_helper import Alignment, Orientation


class TreeTest(unittest.TestCase):

    def test_node(self):
        root_node = Tree()
        root_node.add()
        root_node.add()

        self.assertIsInstance(root_node, Tree)

        self.assertIsInstance(root_node.children[0], Tree)

        for child in root_node.children:
            self.assertIsInstance(child, Tree)

    def test_node_2d(self):
        root_node = Tree2D()

        root_node.add(Orientation.LEFT)

        self.assertIsInstance(root_node.children[Orientation.LEFT], Tree2D)

        try:
            root_node.add(Orientation.LEFT)
            self.assertTrue(False)
        except Exception:
            self.assertTrue(True)

    def test_node_3d(self):
        root_node = Tree3D()

        root_node.add(Alignment.FRONT)

        self.assertIsInstance(root_node.children[Alignment.FRONT], Tree3D)

        try:
            root_node.add(Alignment.FRONT)
            self.assertTrue(False)
        except Exception:
            self.assertTrue(True)
