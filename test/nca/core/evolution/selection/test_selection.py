import unittest
from typing import List

from revolve2.abstract.configurations import PopulationConfiguration
from revolve2.nca.core.actor.actors import Actors
from revolve2.nca.core.actor.individual import Individual
from revolve2.nca.core.actor.individual_factory import ActorFactory
from revolve2.nca.core.ecology.population import Population
from revolve2.nca.core.evolution.selection.parent_selection import NullParentSelection
from revolve2.nca.core.evolution.selection.survivor_selection import NullSurvivorSelection


class TestSelection(unittest.TestCase):

    def test_parent_selection(self):
        population = Population(ActorFactory().create(PopulationConfiguration().population_size))
        parent_selection = NullParentSelection()

        selected = parent_selection.select(population.individuals)

        self.assertIsInstance(selected, List)
        for members in selected:
            self.assertIsInstance(members, List)
        self.assertEqual(len(selected), parent_selection.configuration.selection_size)

    def test_survivor_selection(self):
        population = Population(ActorFactory().create(PopulationConfiguration().population_size))
        survivor_selection = NullSurvivorSelection()

        selected = survivor_selection.select(population.individuals)
        self.assertEqual(len(selected), 2)
