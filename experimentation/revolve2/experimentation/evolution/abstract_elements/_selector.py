from abc import ABC, abstractmethod
from typing import Any

TPopulation = (
    Any  # An alias for Any signifying that a population can vary depending on use-case.
)
KWArgs = dict[str, Any]


class Selector(ABC):
    """A Selector object that enables selection of individuals in an evolutionary process."""

    @abstractmethod
    def select(
        self, population: TPopulation, **kwargs: Any
    ) -> tuple[TPopulation, KWArgs]:
        """
        Select individuals from a population.

        :param population: The population for selection.
        :param kwargs: Possible metrics for selection.
        :return: The selected subset of the population and additional kwargs.
        """
