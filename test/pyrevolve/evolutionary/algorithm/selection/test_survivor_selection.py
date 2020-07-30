import unittest

from pyrevolve.evolutionary import Individual
from pyrevolve.evolutionary.algorithm.selection.survivor_selection import FitnessSteadyStateSelection, \
    ElitismSelection, NullSurvivorSelection, GenerationalSteadyStateSelection
from pyrevolve.evolutionary.ecology.population import Population
from pyrevolve.evolutionary.individual_factory import IndividualFactory
from pyrevolve.shared.configurations import PopulationConfiguration


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