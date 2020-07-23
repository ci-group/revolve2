import unittest
from typing import List

from pyrevolve.evolutionary.individual import Individual
from pyrevolve.evolutionary.agents import Agents


class TestAgents(unittest.TestCase):

    def test_compare(self):

        individual_list = [Individual(), Individual(), Individual()]

        agents = Agents(individual_list)

        for index, agent in enumerate(agents):
            self.assertEqual(agent, individual_list[index])

        self.assertEqual(agents.average_fitness(), 0.0)

    def test_age(self):
        individual_list = [Individual(), Individual(), Individual()]

        agents = Agents(individual_list)

        for index in range(1, 4):
            agents.update_age()
            for agent in agents:
                self.assertEqual(agent.age.generations, index)
