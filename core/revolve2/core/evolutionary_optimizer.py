from abc import ABC, abstractmethod
from typing import Generic, List, Optional, Tuple, TypeVar

from .optimizer import Optimizer

Individual = TypeVar("Individual")
Evaluation = TypeVar("Evaluation")


class EvolutionaryOptimizer(ABC, Optimizer, Generic[Individual, Evaluation]):
    _EvaluatedIndividual = Tuple[Individual, Evaluation]
    _Generation = List[_EvaluatedIndividual]

    _generations: List[_Generation] = []
    _first_generation: Optional[List[Individual]]

    def __init__(
        self,
        initial_population: List[Individual],
        initial_evaluation: Optional[List[Evaluation]],
    ) -> None:
        if initial_evaluation is not None:
            self._generations.append(list(zip(initial_population, initial_evaluation)))
            self._first_generation = None
        else:
            self._first_generation = initial_population

    @abstractmethod
    async def _evaluate_generation(
        self, individuals: List[Individual]
    ) -> List[Evaluation]:
        """
        Evaluate an individual.

        :param individual: The individual to evaluate. Must not be altered.
        :return: The evaluation result.
        """

    @abstractmethod
    def _select_parents(
        self, generation: List[Tuple[Individual, Evaluation]]
    ) -> List[List[Individual]]:
        """
        Selects sets of parents from the given generation
        that will make children for the next generation.

        :param population: The generation to select sets of parents from. Must not be altered.
        :return: The selected sets of parents.
        """

    @abstractmethod
    def _crossover(self, parents: List[Individual]) -> Individual:
        """
        Combine a set of individuals into a new individual.

        :param parents: The set of individuals to combine. Must not be altered.
        :return: The new individual.
        """

    @abstractmethod
    def _mutate(self, individual: Individual) -> Individual:
        """
        Apply mutation to an individual to create a new individual.

        :param individual: The original individual. Must not be altered.
        :return: The new individual.
        """

    async def run_until_completion(self) -> None:
        while await self.process_next_generation():
            pass

    async def evaluate_first_generation(self) -> None:
        if self._first_generation is not None:
            evaluation = await self._evaluate_generation(self._first_generation)
            self._generations.append(list(zip(self._first_generation, evaluation)))
            self._first_generation = None

    async def process_next_generation(self) -> bool:
        await self.evaluate_first_generation()
        parent_selections: List[List[Individual]] = self._select_parents(
            self._generations[-1]
        )
        offspring = [
            self._mutate(self._crossover(selection)) for selection in parent_selections
        ]
        evaluation = await self._evaluate_generation(offspring)
        self._generations.append(list(zip(offspring, evaluation)))
        return False  # TODO stop condition

    @property
    def generation_index(self) -> int:
        """
        Get the current generation.
        The initial generation is numbered 0.
        """

        return len(self._generations) - 1

    @property
    def generations(self) -> List[_Generation]:
        return self._generations
