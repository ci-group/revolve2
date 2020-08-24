import unittest

from evosphere.mock_evosphere import MockEvosphere
from revolve.evosphere.evosphere import GeneticEvosphere


class TestEvosphere(unittest.TestCase):

    def test_create(self):

        evosphere = MockEvosphere()
        evosphere.evolve()
        self.assertTrue(True)

    def test_genetic(self):
        evosphere = GeneticEvosphere(debug=True)
        evosphere.evolve()
        self.assertTrue(True)
