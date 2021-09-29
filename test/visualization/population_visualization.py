import unittest

from revolve2.nca.evolution import Evolution
from revolve2.visualization.population_visualization import PopulationFitnessVisualization


class TestRepeatedEvolution(unittest.TestCase):

    def test_create(self):
        evolution = Evolution()
        population_ecology = evolution.evolve()

        visualization = PopulationFitnessVisualization(population=population_ecology.management.population)
        visualization.prepare()
        visualization.visualize()
