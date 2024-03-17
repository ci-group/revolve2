from abc import ABC, abstractmethod
from typing import Any

TPopulation = (
    Any  # An alias for Any signifying that a population can vary depending on use-case.
)


class Evaluator(ABC):
    """An Evaluator object that enables evaluation of individuals in an evolutionary process."""

    @abstractmethod
    def evaluate(self, population: TPopulation) -> list[float]:
        """
        Evaluate individuals from a population.

        :param population: The population for evaluation.
        :return: The results of the evaluation.
        """
