import unittest

from nca.core.actor.individual_factory import ActorFactory
from nca.evolution import Evolution
from visualization.population_visualization import PopulationVisualization, PopulationFitnessVisualization
from visualization.visualization import time_series_visualization


class TestEvolution(unittest.TestCase):

    def test_create(self):
        evolution = Evolution()
        evolution.evolve()
        self.assertTrue(True)

    def test_metrics(self):
        evolution = Evolution(debug=True)
        population_ecology = evolution.evolve()
        print(population_ecology.management.population.population_metrics)
        print(population_ecology.management.population.individual_metrics)
        time_series_visualization(population_ecology.management.population.population_metrics)
        viz = PopulationFitnessVisualization(population_ecology.management.population)
        viz.prepare()
        viz.visualize()