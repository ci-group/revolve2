import unittest
from typing import List

from pyrevolve.evolutionary.agents import Agents
from pyrevolve.evolutionary.algorithm.selection.parent_selection import NullParentSelection
from pyrevolve.evolutionary.algorithm.selection.survivor_selection import NullSurvivorSelection
from pyrevolve.evolutionary.ecology.population import Population
from pyrevolve.evolutionary.individual_factory import IndividualFactory
from pyrevolve.shared.configurations import PopulationConfiguration


class TestSelection(unittest.TestCase):

    def test_parent_selection(self):
        population = Population(IndividualFactory().create(n=PopulationConfiguration().population_size))
        parent_selection = NullParentSelection()

        selected = parent_selection.select(population.individuals)

        self.assertIsInstance(selected, List)
        for members in selected:
            self.assertIsInstance(members, Agents)
        self.assertEqual(len(selected), parent_selection.configuration.selection_size)

    def test_survivor_selection(self):
        population = Population(IndividualFactory().create(n=PopulationConfiguration().population_size))
        survivor_selection = NullSurvivorSelection()

        selected = survivor_selection.select(population.individuals)

        self.assertIsInstance(selected, Agents)
        self.assertEqual(len(selected), survivor_selection.configuration.population_size)
