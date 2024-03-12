from abc import ABC, abstractmethod
from typing import Any

Population = Any  # An alias for Any to make it easier for people to understand.
KWArgs = dict[str, Any]


class Selector(ABC):
    """A Selector object that enables selection of individuals in an evolutionary process."""

    @abstractmethod
    def select(
        self, population: Population, **kwargs: Any
    ) -> tuple[Population, KWArgs]:
        """
        Select individuals from a population.

        :param population: The population for selection.
        :param kwargs: Possible metrics for selection.
        :return: The selected subset of the population and additional kwargs.
        """
