import unittest
from nca.core.agent.agents import Agents
from nca.core.agent.individual_factory import IndividualFactory


class TestAgents(unittest.TestCase):

    def test_compare(self):

        individual_list = IndividualFactory().create(3)

        agents = Agents(individual_list)

        for index, agent in enumerate(agents):
            self.assertEqual(agent, individual_list[index])

        self.assertEqual(agents.average_fitness(), 0.0)

    def test_age(self):
        individual_list = IndividualFactory().create(3)

        agents = Agents(individual_list)

        for index in range(1, 4):
            agents.update_age()
            for agent in agents:
                self.assertEqual(agent.age.generations, index)
