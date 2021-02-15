import unittest

from nca.evolution import Evolution
from visualization.population_visualization import PopulationFitnessVisualization


class TestRepeatedEvolution(unittest.TestCase):

    def test_create(self):
        evolution = Evolution()
        population_ecology = evolution.evolve()

        visualization = PopulationFitnessVisualization(population=population_ecology.management.population)
        visualization.prepare()
        visualization.visualize()