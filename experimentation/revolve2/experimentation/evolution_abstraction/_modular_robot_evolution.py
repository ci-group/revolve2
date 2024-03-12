from typing import Any

from ._evaluator import Evaluator
from ._learner import Learner
from ._reproducer import Reproducer
from ._selector import Selector

Population = Any  # An alias for Any to make it easier for people to understand.


class ModularRobotEvolution:
    """An object to encapsulate the general functionality of an evolutionary process for modular robots."""

    _parent_selection: Selector
    _survivor_selection: Selector
    _learner: Learner | None
    _evaluator: Evaluator
    _reproducer: Reproducer

    def __init__(
        self,
        parent_selection: Selector,
        survivor_selection: Selector,
        evaluator: Evaluator,
        reproducer: Reproducer,
        learner: Learner | None = None,
    ) -> None:
        """
        Initialize the ModularRobotEvolution object to make robots evolve.

        :param parent_selection: Selector object for the parents for reproduction.
        :param survivor_selection: Selector object for the survivor selection.
        :param evaluator: Evaluator object for evaluation.
        :param reproducer: The reproducer object.
        :param learner: Learning object for learning.
        """
        self._parent_selection = parent_selection
        self._survivor_selection = survivor_selection
        self._evaluator = evaluator
        self._learner = learner
        self._reproducer = reproducer

    def step(self, population: Population, **kwargs: Any) -> Population:
        r"""
        Step the current evolution by one iteration.

        This implementation follows the following schedule:
            [Parent Selection] ------> [Reproduction]
                   ^                        |
                   |                        |
                   |                       \/
            [Survivor Selection] <--- [Evaluation of Children]
        The schedule can be easily adapted and reorganized for your needs.


        :param population: The current population.
        :param kwargs: Additional keyword arguments to use in the step.
        :return: The population resulting from the step
        """
        parents, parent_kwargs = self._parent_selection.select(population, **kwargs)
        children = self._reproducer.reproduce(parents, **parent_kwargs)
        child_task_performance = self._evaluator.evaluate(children)
        survivors, *_ = self._survivor_selection.select(
            population,
            **kwargs,
            children=children,
            child_task_performance=child_task_performance
        )
        return survivors
