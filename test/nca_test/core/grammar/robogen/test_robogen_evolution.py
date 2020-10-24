import copy
import unittest

from nca.core.genome.operators.mutation_operator import SwapMutation, DeleteMutation, InversionMutation, InsertMutation, \
    ReplaceMutation
from revolve.robot.robogen.robogen_representation import RobogenConstruction


class TestMutationOperators(unittest.TestCase):

    def test_swap_mutation(self):
        mutation = SwapMutation()

        algorithm = RobogenConstruction()
        self.assertTrue(len(algorithm.representation) > 0)

        new_algorithm = copy.deepcopy(algorithm)
        self.assertEqual(algorithm.representation, new_algorithm.representation)

        mutation._mutate(new_algorithm.representation)
        self.assertTrue(len(algorithm.representation) == len(new_algorithm.representation))
        self.assertNotEqual(algorithm.representation, new_algorithm.representation)

    def test_delete_mutation(self):
        mutation = DeleteMutation()

        algorithm = RobogenConstruction()
        self.assertTrue(len(algorithm.representation) > 0)

        new_algorithm = copy.deepcopy(algorithm)
        self.assertEqual(algorithm.representation, new_algorithm.representation)

        mutation._mutate(new_algorithm.representation)
        self.assertTrue(len(algorithm.representation) == len(new_algorithm.representation) + 1)
        self.assertNotEqual(algorithm.representation, new_algorithm.representation)

    def test_inversion_mutation(self):
        mutation = InversionMutation()

        algorithm = RobogenConstruction()
        self.assertTrue(len(algorithm.representation) > 0)

        new_algorithm = copy.deepcopy(algorithm)
        self.assertEqual(algorithm.representation, new_algorithm.representation)

        mutation._mutate(new_algorithm.representation)
        self.assertTrue(len(new_algorithm.representation) == len(algorithm.representation))
        self.assertNotEqual(algorithm.representation, new_algorithm.representation)

    def test_insert_mutation(self):
        mutation = InsertMutation()

        algorithm = RobogenConstruction()
        self.assertTrue(len(algorithm.representation) > 0)

        new_algorithm = copy.deepcopy(algorithm)
        self.assertEqual(algorithm.representation, new_algorithm.representation)

        mutation._mutate(new_algorithm.representation)
        self.assertTrue(len(new_algorithm.representation) == len(algorithm.representation) + 1)
        self.assertNotEqual(algorithm.representation, new_algorithm.representation)

    def test_replace_mutation(self):
        mutation = ReplaceMutation()

        algorithm = RobogenConstruction()
        self.assertTrue(len(algorithm.representation) > 0)

        new_algorithm = copy.deepcopy(algorithm)
        self.assertEqual(algorithm.representation, new_algorithm.representation)

        mutation._mutate(new_algorithm.representation)
        self.assertTrue(len(new_algorithm.representation) == len(algorithm.representation))
        self.assertNotEqual(algorithm.representation, new_algorithm.representation)
