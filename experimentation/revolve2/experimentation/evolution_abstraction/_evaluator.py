from abc import ABC, abstractmethod
from typing import Any

TPopulation = Any  # An alias for Any to make it easier for people to understand.


class Evaluator(ABC):
    """An Evaluator object that enables evaluation of individuals in an evolutionary process."""

    @abstractmethod
    def evaluate(self, population: TPopulation) -> list[float]:
        """
        Evaluate individuals from a population.

        :param population: The population for evaluation.
        :return: The results of the evaluation.
        """
