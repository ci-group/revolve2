import unittest
from typing import List

from nca.core.abstract.configurations import PopulationConfiguration
from nca.core.actor.actors import Actors
from nca.core.actor.individual_factory import IndividualFactory
from nca.core.ecology.population import Population
from nca.core.evolution.selection.parent_selection import NullParentSelection
from nca.core.evolution.selection.survivor_selection import NullSurvivorSelection


class TestSelection(unittest.TestCase):

    def test_parent_selection(self):
        population = Population(IndividualFactory().create(PopulationConfiguration().population_size))
        parent_selection = NullParentSelection()

        selected = parent_selection.select(population.individuals)

        self.assertIsInstance(selected, List)
        for members in selected:
            self.assertIsInstance(members, Actors)
        self.assertEqual(len(selected), parent_selection.configuration.selection_size)

    def test_survivor_selection(self):
        population = Population(IndividualFactory().create(PopulationConfiguration().population_size))
        survivor_selection = NullSurvivorSelection()

        selected = survivor_selection.select(population.individuals)

        self.assertIsInstance(selected, Actors)
        self.assertEqual(len(selected), survivor_selection.configuration.population_size)
