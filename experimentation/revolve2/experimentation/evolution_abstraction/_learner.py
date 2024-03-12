from abc import ABC, abstractmethod
from typing import TypeVar

TPopulation = TypeVar('TPopulation')


class Learner(ABC):
    """A Learner object that enables learning for individuals in an evolutionary process."""

    @abstractmethod
    def learn(self, population: TPopulation) -> TPopulation:
        """
        Make Individuals from a population learn.

        :param population: The population.
        :return: The learned population.
        """
