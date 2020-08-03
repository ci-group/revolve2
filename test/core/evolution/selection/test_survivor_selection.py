import unittest

from nca.experiment.configurations import PopulationConfiguration
from nca.core.agent import Individual
from nca.core.evolution.selection.survivor_selection import FitnessSteadyStateSelection, \
    ElitismSelection, NullSurvivorSelection, GenerationalSteadyStateSelection
from nca.core.ecology.population import Population
from nca.core.agent.individual_factory import IndividualFactory


class TestSurvivorSelection(unittest.TestCase):

    def test_random_selection(self):
        population = Population(IndividualFactory().create(n=PopulationConfiguration().population_size))

        steady_state = NullSurvivorSelection()

        selected = steady_state.algorithm(population.individuals)

        for element in selected:
            self.assertIsInstance(element, Individual)

        self.assertEqual(len(selected), len(population.individuals))

    def test_fitness_selection(self):
        population = Population(IndividualFactory().create(n=PopulationConfiguration().population_size))

        steady_state = FitnessSteadyStateSelection()

        selected = steady_state.algorithm(population.individuals)

        for element in selected:
            self.assertIsInstance(element, Individual)

        self.assertEqual(len(selected), len(population.individuals))

    def test_age_selection(self):
        population = Population(IndividualFactory().create(n=PopulationConfiguration().population_size))

        steady_state = GenerationalSteadyStateSelection()

        selected = steady_state.algorithm(population.individuals)

        for element in selected:
            self.assertIsInstance(element, Individual)

        self.assertEqual(len(selected), len(population.individuals))

    def test_elitism_selection(self):
        population = Population(IndividualFactory().create(n=PopulationConfiguration().population_size))

        steady_state = ElitismSelection()

        selected = steady_state.algorithm(population.individuals)

        for element in selected:
            self.assertIsInstance(element, Individual)

        self.assertEqual(len(selected), len(population.individuals))