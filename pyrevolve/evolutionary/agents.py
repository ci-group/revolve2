from typing import List, Iterator

from pyrevolve.evolutionary.agent import Agent
from pyrevolve.evolutionary.fitness import Fitness


class Agents(Iterator):

    def __init__(self, agents: List[Agent] = []):
        self.agents: List[Agent] = agents
        self.last_best_agent: Agent = None
        self.last_worst_agent: Agent = None
        self.index = 0

    def add(self, agent: Agent):
        self.agents.append(agent)

    def remove(self, agent: Agent):
        self.agents.remove(agent)

    def get_best(self):
        if self.last_best_agent is not None:
            return self.last_best_agent

        best_agent = None
        best_fitness = Fitness.worst()

        for agent in self.agents:
            if best_fitness > agent.fitness:
                best_fitness = agent.fitness
                best_agent = agent

        self.last_best_agent = best_agent

        return best_agent

    def get_worst(self):
        if self.last_worst_agent is not None:
            return self.last_worst_agent

        worst_agent = None
        worst_fitness = Fitness.best()

        for agent in self.agents:
            if worst_fitness < agent.fitness:
                worst_fitness = agent.fitness
                worst_agent = agent

        self.last_worst_agent = worst_agent

        return worst_agent

    def __len__(self):
        return len(self.agents)

    def __getitem__(self, index):
        return self.agents[index]

    def __iter__(self):
        self.index = 0
        return self

    def __next__(self) -> Agent:
        try:
            agent = self.agents[self.index]
            self.index += 1
        except IndexError:
            raise StopIteration()

        return agent
