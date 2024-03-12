from ._selector import Selector
from ._learner import Learner
from ._evaluator import Evaluator
from ._reproducer import Reproducer
from typing import TypeVar

TPopulation = TypeVar('TPopulation')


class ModularRobotEvolution:
    """An object to encapsulate the general functionality of an evolutionary process for modular robots."""
    _parent_selection: Selector
    _survivor_selection: Selector
    _learner: Learner | None
    _evaluator: Evaluator
    _reproducer: Reproducer

    def __init__(self, parent_selection: Selector, survivor_selection: Selector, evaluator: Evaluator, reproducer: Reproducer,
                 learner: Learner | None = None) -> None:
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

    def step(self, population: TPopulation) -> TPopulation:
        """
        Step the current evolution by one iteration.

        :return: The population resulting from the step
        """
        task_performance = self._evaluator.evaluate(population)
        parents = self._parent_selection.select(population, task_performance)
        children = self._reproducer.reproduce(parents)
        new_population = parents + children
        child_task_performance = self._evaluator.evaluate(children)

        survivors = self._survivor_selection(population)



