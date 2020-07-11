from random import randint, random

from pyrevolve.evolutionary.algorithm.selection import ParentSelection
from pyrevolve.evolutionary.agent import Agent
from pyrevolve.evolutionary.agents import Agents


class RandomSelection(ParentSelection):

    def __init__(self):
        super().__init__()

    def algorithm(self, agents: Agents) -> Agent:
        return agents[randint(0, len(agents) - 1)]


class TournamentSelection(ParentSelection):

    def __init__(self):
        super().__init__()

    def algorithm(self, agents: Agents) -> Agent:
        best_agent: Agent = None

        for _ in range(self.configuration.tournament_size):
            agent = random.choice(agents)
            # probability to reconsider choice.

            if (best_agent is None) or (agent.fitness > best_agent.fitness):
                best_agent = agent

        return best_agent


class RouletteWheelSelection(ParentSelection):

    def __init__(self):
        super().__init__()

    def algorithm(self, agents: Agents) -> Agent:
        best_agent: Agent = None

        for _ in range(self.configuration.tournament_size):
            agent = random.choice(agents)
            # probability to reconsider choice.

            if (best_agent is None) or (agent.fitness > best_agent.fitness):
                best_agent = agent

        return best_agent

