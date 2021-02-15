import unittest

from abstract.structural.tree.tree import Tree


class TreeTest(unittest.TestCase):

    def test_node(self):
        root_node = Tree()
        root_node.add()
        root_node.add()

        self.assertIsInstance(root_node, Tree)

        self.assertIsInstance(root_node.children[0], Tree)

        for child in root_node.children:
            self.assertIsInstance(child, Tree)
