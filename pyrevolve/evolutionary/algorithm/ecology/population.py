from pyrevolve.evolutionary.algorithm.ecology.age import Age
from pyrevolve.evolutionary.algorithm.ecology import Compatibility
from pyrevolve.evolutionary.agent import Agent
from pyrevolve.evolutionary.agents import Agents, Offspring
from pyrevolve.evolutionary.things.performance_measures import PerformanceMeasures


class Population:

    compatibility: Compatibility = Compatibility()

    def __init__(self, population_id: int, agents: Agents):
        # Basics
        self.id: int = population_id
        self.parents: Agents = agents
        self.age: Age = Age()

        # Speciation
        self.representative = None
        self.offspring: Offspring = Offspring()

    def __get__(self):
        return self.parents

    def compatible(self, agent: Agent):
        if self.compatibility(self.parents, agent):
            self.parents.add(agent)
            return True

        return False

    def average_fitness(self):
        average_performance = PerformanceMeasures()
        for agent in self.parents:
            agent.fitness


        return