from abc import ABC, abstractmethod
from typing import TypeVar, Any

TPopulation = TypeVar('TPopulation')


class Selector(ABC):
    """A Selector object that enables selection of individuals in an evolutionary process."""

    @abstractmethod
    def select(self, population: TPopulation, *metrics: Any) -> TPopulation:
        """
        Select individuals from a population.

        :param population: The population for selection.
        :param metrics: Possible metrics for selection.
        :return: The selected subset of the population.
        """
