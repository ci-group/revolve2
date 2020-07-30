import unittest
from typing import List

from pyrevolve.evolutionary import Individual
from pyrevolve.evolutionary.algorithm.selection.parent_selection import TournamentSelection, RandomParentSelection, \
    RouletteWheelSelection
from pyrevolve.evolutionary.ecology.population import Population
from pyrevolve.evolutionary.individual_factory import IndividualFactory
from pyrevolve.shared.configurations import PopulationConfiguration


class TestParentSelection(unittest.TestCase):

    def test_random_selection(self):
        population = Population(IndividualFactory().create(n=PopulationConfiguration().population_size))

        tournament_selection = RandomParentSelection()

        selected = tournament_selection.algorithm(population.individuals)

        self.assertIsInstance(selected, List)
        for individual in selected:
            self.assertIsInstance(individual, Individual)

    def test_tournament_selection(self):
        population = Population(IndividualFactory().create(n=PopulationConfiguration().population_size))

        tournament_selection = TournamentSelection()

        selected = tournament_selection.algorithm(population.individuals)

        self.assertIsInstance(selected, List)
        for individual in selected:
            self.assertIsInstance(individual, Individual)

    def test_roulette_selection(self):
        population = Population(IndividualFactory().create(n=PopulationConfiguration().population_size))

        tournament_selection = RouletteWheelSelection()

        selected = tournament_selection.algorithm(population.individuals)

        self.assertIsInstance(selected, List)
        for individual in selected:
            self.assertIsInstance(individual, Individual)
