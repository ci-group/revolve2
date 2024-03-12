from abc import ABC, abstractmethod
from typing import TypeVar

TPopulation = TypeVar('TPopulation')


class Reproducer(ABC):
    """A Reproducer object that enables the reproduction of individuals in an evolutionary process."""

    @abstractmethod
    def reproduce(self, population: TPopulation) -> TPopulation:
        """
        Make Individuals Reproduce.

        :param population: The population.
        :return: The children.
        """
