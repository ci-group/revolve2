import unittest

from revolve2.abstract.configurations import PopulationConfiguration
from revolve2.nca.core.actor.individual import Individual
from revolve2.nca.core.actor.individual_factory import ActorFactory
from revolve2.nca.core.ecology.population import Population
from revolve2.nca.core.evolution.selection.survival_non_dominated import NonDominatedSortingSurvival
from revolve2.nca.core.evolution.selection.survivor_selection import NullSurvivorSelection, FitnessSteadyStateSelection, \
    GenerationalSteadyStateSelection, ElitismSelection
import numpy as np


class TestSurvivorSelection(unittest.TestCase):

    def test_random_selection(self):
        population = Population(ActorFactory().create(PopulationConfiguration().population_size))

        steady_state = NullSurvivorSelection()

        selected = steady_state.algorithm(population.individuals)

        for element in selected:
            self.assertIsInstance(element, Individual)

        self.assertEqual(len(selected), len(population.individuals))

    def test_fitness_selection(self):
        population = Population(ActorFactory().create(PopulationConfiguration().population_size))

        steady_state = FitnessSteadyStateSelection()

        selected = steady_state.algorithm(population.individuals)

        for element in selected:
            self.assertIsInstance(element, Individual)

        self.assertEqual(len(selected), len(population.individuals))

    def test_age_selection(self):
        population = Population(ActorFactory().create(PopulationConfiguration().population_size))

        steady_state = GenerationalSteadyStateSelection()

        selected = steady_state.algorithm(population.individuals)

        for element in selected:
            self.assertIsInstance(element, Individual)

        self.assertEqual(len(selected), len(population.individuals))

    def test_elitism_selection(self):
        population = Population(ActorFactory().create(PopulationConfiguration().population_size))

        steady_state = ElitismSelection()

        selected = steady_state.algorithm(population.individuals)

        for element in selected:
            self.assertIsInstance(element, Individual)

        self.assertEqual(len(selected), len(population.individuals))

    def test_non_dominated_selection(self):
        population = Population(ActorFactory().create(PopulationConfiguration().population_size))

        for individual in population.individuals:
            individual.fitness.add("test1", np.random.random())
            individual.fitness.add("test2", np.random.random())

        steady_state = NonDominatedSortingSurvival()

        selected = steady_state.algorithm(population.individuals)

        for element in selected:
            self.assertIsInstance(element, Individual)

        self.assertEqual(len(selected), len(population.individuals))
