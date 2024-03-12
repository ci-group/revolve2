from abc import ABC, abstractmethod
from typing import Any

TPopulation = Any  # An alias for Any to make it easier for people to understand.


class Reproducer(ABC):
    """A Reproducer object that enables the reproduction of individuals in an evolutionary process."""

    @abstractmethod
    def reproduce(self, population: TPopulation, **kwargs: Any) -> TPopulation:
        """
        Make Individuals Reproduce.

        :param population: The population.
        :param kwargs: Additional arguments.
        :return: The children.
        """
