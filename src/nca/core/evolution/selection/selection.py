
# Multiple Selection
from abc import ABC, abstractmethod
from typing import List

from nca.core.abstract.behavioral.command import Command
from nca.core.abstract.configurations import PopulationConfiguration
from nca.core.actor.actors import Actors
from nca.core.actor.individual import Individual


class Selection(Command, ABC):

    def __init__(self):
        self.configuration = PopulationConfiguration()

    def check(self, individuals: Actors):
        number_of_agents = len(individuals)
        assert self.configuration.selection_size <= number_of_agents
        assert self.configuration.population_size <= number_of_agents
        assert self.configuration.tournament_size <= self.configuration.population_size


class ParentSelection(Selection, ABC):

    def __init__(self):
        super().__init__()

    def select(self, individuals: Actors) -> List[Actors]:
        self.check(individuals)
        return [Actors(self.algorithm(individuals)) for _ in range(self.configuration.selection_size)]

    @abstractmethod
    def algorithm(self, individuals: Actors) -> List[Individual]:
        raise Exception("Parent selection evolution is not implemented")


class SurvivorSelection(Selection, ABC):

    def __init__(self):
        super().__init__()

    def select(self, individuals: Actors) -> Actors:
        super().check(individuals)
        # TODO simplify collection
        new_individuals: List[Individual] = self(individuals)     # mmhhh..
        return Actors(new_individuals[:self.configuration.population_size])

    @abstractmethod
    def __call__(self, individuals: Actors) -> List[Individual]:
        raise Exception("Survivor selection evolution is not implemented")


class MortalitySelection(Selection, ABC):

    def __init__(self):
        super().__init__()
        self.mortality_percentage = 0.05

    @abstractmethod
    def __call__(self, individuals: Actors, offspring: Actors):
        raise Exception("Survivor selection evolution is not implemented")

