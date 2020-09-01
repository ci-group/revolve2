import unittest

from nca.evolution import Evolution


class TestEvolution(unittest.TestCase):

    def test_create(self):
        evolution = Evolution()
        evolution.evolve()
        self.assertTrue(True)
