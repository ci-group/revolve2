from pyrevolve.evolutionary.algorithm.ecology.age import Age
from pyrevolve.evolutionary.algorithm.ecology import Compatibility
from pyrevolve.evolutionary.agent import Agent
from pyrevolve.evolutionary.agents import Agents
from pyrevolve.evolutionary.things.performance_measures import PerformanceMeasures


class Population:

    compatibility: Compatibility = Compatibility()

    def __init__(self, population_id: int, agents: Agents):
        # Basics
        self.id: int = population_id
        self.agents: Agents = agents
        self.age: Age = Age()

        # Speciation
        self.representative = None

    def __get__(self):
        return self.agents

    def compatible(self, agent: Agent):
        if self.compatibility(self.agents, agent):
            self.agents.add(agent)
            return True

        return False

    def average_fitness(self):
        average_performance = PerformanceMeasures()
        for agent in self.agents:
            agent.fitness


        return