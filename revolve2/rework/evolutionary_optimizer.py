from dataclasses import dataclass
from typing import Any, Callable, Generic, List, Optional, Tuple, TypeVar

from .optimizer import Optimizer

_individual_type = TypeVar("_individual_type")
_evaluation_type = TypeVar("_evaluation_type")


class EvolutionaryOptimizer(Optimizer, Generic[_individual_type, _evaluation_type]):
    _EvaluatedIndividual = Tuple[_individual_type, _evaluation_type]
    _Generation = List[_EvaluatedIndividual]

    _generations: List[_Generation] = []
    _first_generation: Optional[List[_individual_type]]

    _evaluate: Callable[[_individual_type], _evaluation_type]
    _select_parents: Callable[
        [List[_EvaluatedIndividual]],
        List[List[_individual_type]],
    ]
    _crossover: Callable[[List[_individual_type]], _individual_type]
    _mutate: Callable[[_individual_type], _individual_type]

    def __init__(
        self,
        initial_population: List[_individual_type],
        initial_evaluation: Optional[List[_evaluation_type]],
        evaluate: Callable[[List[_individual_type]], List[_evaluation_type]],
        select_parents: Callable[
            [List[_EvaluatedIndividual]],
            List[List[_individual_type]],
        ],
        crossover: Callable[[List[_individual_type]], _individual_type],
        mutate: Callable[[_individual_type], _individual_type],
    ) -> None:
        if initial_evaluation != None:
            self._generations.append(list(zip(initial_population, initial_evaluation)))
            self._first_generation = None
        else:
            self._first_generation = initial_population

        self._evaluate = evaluate
        self._select_parents = select_parents
        self._crossover = crossover
        self._mutate = mutate

    def run_until_completion(self) -> None:
        while self.process_next_generation():
            pass

    def evaluate_first_generation(self) -> None:
        if self._first_generation != None:
            evaluation = [
                self._evaluate(individual) for individual in self._first_generation
            ]
            self._generations.append(list(zip(self._first_generation, evaluation)))
            self._first_generation = None

    def process_next_generation(self) -> bool:
        self.evaluate_first_generation()
        parent_selections: List[List[_individual_type]] = self._select_parents(
            self._generations[-1]
        )
        offspring = [
            self._mutate(self._crossover(selection)) for selection in parent_selections
        ]
        evaluation = [self._evaluate(individual) for individual in offspring]
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
