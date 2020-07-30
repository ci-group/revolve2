import unittest
from typing import List

from pyrevolve.evolutionary.individual import Individual


class TestIndividual(unittest.TestCase):

    def test_compare(self):

        parent1 = Individual()
        parent2 = Individual()

        parents: List[Individual] = [parent1, parent2]
        child = Individual(parents=parents)

        self.assertTrue(parent1.id != parent2.id)
        self.assertTrue(child.parents.index(parent1) >= 0)
        self.assertTrue(child.parents.index(parent2) >= 0)
