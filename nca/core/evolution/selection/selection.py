
# Multiple Selection
from abc import ABC, abstractmethod
from typing import List

from nca.core.agent.agents import Agents, Individual
from nca.core.abstract.command import Command
from nca.core.abstract.configurations import PopulationConfiguration


class Selection(Command, ABC):

    def __init__(self):
        self.configuration = PopulationConfiguration()

    def check(self, individuals: Agents):
        number_of_agents = len(individuals)
        assert self.configuration.selection_size <= number_of_agents
        assert self.configuration.population_size <= number_of_agents
        assert self.configuration.tournament_size <= self.configuration.population_size


class ParentSelection(Selection, ABC):

    def __init__(self):
        super().__init__()

    def select(self, individuals: Agents) -> List[Agents]:
        self.check(individuals)
        return [Agents(self.algorithm(individuals)) for _ in range(self.configuration.selection_size)]

    @abstractmethod
    def algorithm(self, individuals: Agents) -> List[Individual]:
        raise Exception("Parent selection evolution is not implemented")


class SurvivorSelection(Selection, ABC):

    def __init__(self):
        super().__init__()

    def select(self, individuals: Agents) -> Agents:
        super().check(individuals)
        # TODO simplify collection
        new_individuals: List[Individual] = self.algorithm(individuals)     # mmhhh...
        return Agents(new_individuals[:self.configuration.population_size])

    @abstractmethod
    def algorithm(self, individuals: Agents) -> List[Individual]:
        raise Exception("Survivor selection evolution is not implemented")
