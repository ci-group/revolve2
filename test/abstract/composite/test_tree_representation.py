import unittest

from revolve2.abstract.structural.tree.tree import Tree, CoordinateTree
from revolve2.abstract.structural.tree.tree_helper import Orientation
from revolve2.nca.core.genome.operators.recombination_operator import SwapTreeCrossover
from revolve2.nca.core.genome.representations.tree_representation import TreeRepresentation, CoordinateTreeRepresentation


class TreeRepresentationTest(unittest.TestCase):

    def test_tree(self):
        root_node = Tree()
        root_node.add()
        root_node.add()

        representation = TreeRepresentation(root_node)

        self.assertIsInstance(representation, TreeRepresentation)
        self.assertIsInstance(representation[0], Tree)

    def test_coordinate_tree(self):
        root_node = CoordinateTree()

        representation = CoordinateTreeRepresentation(root_node)

        root_node.add(Orientation.TOP)
        root_node.add(Orientation.DOWN)

        self.assertIsInstance(representation[0], CoordinateTree)
        for key, element in representation[0].children.items():
            self.assertIsInstance(element, CoordinateTree)

    def test_recombination_tree(self):

        coordinate_tree_1 = CoordinateTree()
        coordinate_tree_1.add(Orientation.TOP)
        coordinate_tree_1.add(Orientation.LEFT)
        representation_1 = CoordinateTreeRepresentation(coordinate_tree_1)

        coordinate_tree_2 = CoordinateTree()
        coordinate_tree_2.add(Orientation.DOWN)
        coordinate_tree_2.add(Orientation.RIGHT)
        representation_2 = CoordinateTreeRepresentation(coordinate_tree_2)

        recombination = SwapTreeCrossover()
        recombination._recombine(representation_1, representation_2)

        self.assertIsInstance(representation_1[0], CoordinateTree)
        self.assertIsInstance(representation_2[0], CoordinateTree)

