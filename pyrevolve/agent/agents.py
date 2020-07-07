import math
from abc import ABC
from typing import List

from pyrevolve.agent.agent import Agent


class Agents(ABC):

    def __init__(self, agents: List[Agent]):
        self.agents: List[Agent] = agents
        self.last_best_agent = None

    def __len__(self):
        return len(self.agents)

    def get_best(self):
        if self.last_best_agent is not None:
            return self.last_best_agent

        best_agent = None
        # TODO Fitness Class
        best_fitness = -math.inf

        for agent in self.agents:
            # TODO compare in class
            if best_fitness > agent.fitness.selected_fitness:
                best_fitness = agent.fitness.selected_fitness
                best_agent = agent

        return best_agent

    def get_worst(self):
        if self.last_best_agent is not None:
            return self.last_best_agent

        worst_agent = None
        # TODO Fitness Class
        worst_fitness = math.inf

        for agent in self.agents:
            # TODO compare in class
            if worst_fitness < agent.fitness.selected_fitness:
                worst_fitness = agent.fitness.selected_fitness
                worst_agent = agent

        return worst_agent
