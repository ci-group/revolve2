
# Multiple Selection
from abc import ABC, abstractmethod
from typing import List

from pyrevolve.evolutionary import Agents, Individual
from pyrevolve.shared.abstract.command import Command
from pyrevolve.shared.configurations import PopulationConfiguration


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

    def select(self, individuals: Agents):
        self.check(individuals)
        # TODO simplify
        return Agents([self.algorithm(individuals) for _ in range(self.configuration.selection_size)])

    @abstractmethod
    def algorithm(self, individuals: List[Individual]) -> Individual:
        raise Exception("Parent selection algorithm is not implemented")


class SurvivorSelection(Selection, ABC):

    def __init__(self):
        super().__init__()

    def select(self, individuals: Agents):
        super().check(individuals)
        # TODO simplify collection
        new_individuals: List[Individual] = self.algorithm(individuals) # mmhhh...
        return Agents(new_individuals[:self.configuration.population_size])

    @abstractmethod
    def algorithm(self, individuals: List[Individual]) -> List[Individual]:
        raise Exception("Survivor selection algorithm is not implemented")
