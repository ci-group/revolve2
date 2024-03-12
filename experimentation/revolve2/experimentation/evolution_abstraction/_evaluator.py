from abc import ABC, abstractmethod
from typing import TypeVar

TPopulation = TypeVar('TPopulation')


class Evaluator(ABC):
    """An Evaluator object that enables evaluation of individuals in an evolutionary process."""

    @abstractmethod
    def evaluate(self, population: TPopulation) -> list[float]:
        """
        Evaluate individuals from a population.

        :param population: The population for evaluation.
        :return: The results of the evaluation.
        """
