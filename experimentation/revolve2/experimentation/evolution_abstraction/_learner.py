from abc import ABC, abstractmethod
from typing import Any

Population = Any  # An alias for Any to make it easier for people to understand.


class Learner(ABC):
    """A Learner object that enables learning for individuals in an evolutionary process."""

    @abstractmethod
    def learn(self, population: Population) -> Population:
        """
        Make Individuals from a population learn.

        :param population: The population.
        :return: The learned population.
        """
