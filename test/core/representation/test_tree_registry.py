import unittest

from nca.core.genome.representations.tree import Node2D, Orientation
from nca.core.genome.representations.tree_registry import BiDict, Coordinate3D, tree_registry


class TreeRepresentationTest(unittest.TestCase):

    def test_overlap(self):

        root = Node2D()
        child01 = Node2D()
        child10 = Node2D()
        child11 = Node2D()
        child10.add(child11, Orientation.RIGHT)
        child01.add(child10, Orientation.DOWN)
        root.add(child01, Orientation.LEFT)

        child2 = Node2D()
        root.add(child2, Orientation.DOWN)

        used: BiDict = BiDict()

        used[root.id] = Coordinate3D(0, 0, 0)
        try:
            tree_registry(set(), root.graph(), root, used)
            self.assertTrue(False)
        except Exception:
            self.assertTrue(True)


    def test_node(self):

        root = Node2D()
        child01 = Node2D()
        child10 = Node2D()
        child11 = Node2D()
        child10.add(child11, Orientation.RIGHT)
        child01.add(child10, Orientation.DOWN)
        root.add(child01, Orientation.LEFT)

        child2 = Node2D()
        root.add(child2, Orientation.TOP)

        used: BiDict = BiDict()

        used[root.id] = Coordinate3D(0, 0, 0)
        try:
            tree_registry(set(), root.graph(), root, used)
            self.assertTrue(False)
        except Exception:
            self.assertTrue(True)
