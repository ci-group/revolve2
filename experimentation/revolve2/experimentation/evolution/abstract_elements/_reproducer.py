from abc import ABC, abstractmethod
from typing import Any

TPopulation = (
    Any  # An alias for Any signifying that a population can vary depending on use-case.
)


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
