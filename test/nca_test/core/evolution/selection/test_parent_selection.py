import unittest
from typing import List

from nca.core.abstract.configurations import PopulationConfiguration
from nca.core.actor.individual import Individual
from nca.core.actor.individual_factory import ActorFactory
from nca.core.ecology.population import Population
from nca.core.evolution.selection.parent_selection import RandomParentSelection, TournamentSelection, \
    RouletteWheelSelection


class TestParentSelection(unittest.TestCase):

    def test_random_selection(self):
        population = Population(ActorFactory().create(PopulationConfiguration().population_size))

        tournament_selection = RandomParentSelection()

        selected = tournament_selection.algorithm(population.individuals)

        self.assertIsInstance(selected, List)
        for individual in selected:
            self.assertIsInstance(individual, Individual)

    def test_tournament_selection(self):
        population = Population(ActorFactory().create(PopulationConfiguration().population_size))

        tournament_selection = TournamentSelection()

        selected = tournament_selection.algorithm(population.individuals)

        self.assertIsInstance(selected, List)
        for individual in selected:
            self.assertIsInstance(individual, Individual)

    def test_roulette_selection(self):
        population = Population(ActorFactory().create(PopulationConfiguration().population_size))

        tournament_selection = RouletteWheelSelection()

        selected = tournament_selection.algorithm(population.individuals)

        self.assertIsInstance(selected, List)
        for individual in selected:
            self.assertIsInstance(individual, Individual)
