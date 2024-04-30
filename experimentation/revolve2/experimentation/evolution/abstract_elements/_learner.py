from abc import ABC, abstractmethod
from typing import Any

from ._evaluator import Evaluator

TPopulation = (
    Any  # An alias for Any signifying that a population can vary depending on use-case.
)


class Learner(ABC):
    """
    A Learner object that enables learning for individuals in an evolutionary process.

    The learner is dependent on its reward function, which is: a measure that drives learning.
    Depending on the learning method used, the reward can simply equal task performance.

    Task performance on the other hand is a measure that reflects how well a task is performed.
    In a robot system with multiple tasks, there are multiple definitions of task performance.
    Task performance can be used to define fitness and/or reward functions.

    For more information wait for Prof. De. A.E. Eiben`s book on evolutionary robotics, or ask him directly.
    """

    _reward_function: Evaluator

    @abstractmethod
    def learn(self, population: TPopulation) -> TPopulation:
        """
        Make Individuals from a population learn.

        :param population: The population.
        :return: The learned population.
        """
