from abc import ABC, abstractmethod
from typing import Any, Generic, List, Optional, Tuple, Type, TypeVar

Individual = TypeVar("Individual")
Evaluation = TypeVar("Evaluation")


class EvolutionaryOptimizer(ABC, Generic[Individual, Evaluation]):
    # Types of individual and evaluation are stored as soon as they are available.
    # Used to type check the return values of user functions.
    _individual_type: Type
    _evaluation_type: Optional[Type]

    _population_size: int
    _offspring_size: int

    _generations: List[List[Tuple[Individual, Evaluation]]]
    _first_generation: Optional[List[Individual]]

    def __init__(
        self,
        population_size: int,
        offspring_size: int,
        initial_population: List[Individual],
        initial_evaluation: Optional[List[Evaluation]],
    ) -> None:
        assert type(population_size) == int
        self._population_size = population_size

        assert type(offspring_size) == int
        self._offspring_size = offspring_size

        assert type(initial_population) == list
        assert len(initial_population) == self._population_size
        assert len(initial_population) >= 1

        # save individual type so we can type check things later
        self._individual_type = type(initial_population[0])

        if initial_evaluation is not None:
            assert type(initial_evaluation) == list
            assert len(initial_evaluation) == self._population_size
            self._generations = [list(zip(initial_population, initial_evaluation))]
            self._first_generation = None

            # save evaluation type so we can type check things later
            self._evaluation_type = type(initial_evaluation[0])
        else:
            self._first_generation = initial_population
            self._generations = []

            # set evaluation type to None
            # we will set it when evaluating the first generation later
            self._evaluation_type = None

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
        self, generation: List[Tuple[Individual, Evaluation]], num_parents: int
    ) -> List[List[Tuple[Individual, Evaluation]]]:
        """
        Select sets of parents from the given generation
        that will make children for the next generation.

        :param population: The generation to select sets of parents from. Must not be altered.
        :return: The selected sets of parents.
        """

    def _select_survivors(
        self,
        old_individuals: List[Tuple[Individual, Evaluation]],
        new_individuals: List[Tuple[Individual, Evaluation]],
        num_survivors: int,
    ) -> List[Tuple[Individual, Evaluation]]:
        """
        Select survivors from a group of individuals. These will form the next generation.

        :param individuals: The individuals to choose from.
        :param num_survivors: How many individuals should be selected.
        :returns: The selected individuals.
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

    @abstractmethod
    def _must_do_next_gen(self) -> bool:
        """
        Decide if the optimizer must do another generation.
        :returns: True if it must.
        """

    async def run_until_completion(self) -> None:
        while await self.process_next_generation():
            pass

    async def evaluate_first_generation(self) -> None:
        """
        Evaluate first generation if this is not yet done.
        """

        if self._first_generation is not None:
            # let user evaluate
            evaluation = await self._evaluate_generation(self._first_generation)

            # assert user return value
            assert type(evaluation) == list
            assert len(evaluation) == len(self._first_generation)
            assert all(type(e) == type(evaluation[0]) for e in evaluation)

            # save evaluation type so we can type check things later
            self._evaluation_type = type(evaluation[0])

            # combine provided individuals and new evaluation to create the first generation
            self._generations.append(list(zip(self._first_generation, evaluation)))
            self._first_generation = None

    async def process_next_generation(self) -> bool:
        # evaluate first generation if this is not yet done
        await self.evaluate_first_generation()

        # let user select parents
        parent_selections: List[
            List[Tuple[Individual, Evaluation]]
        ] = self._select_parents(self._generations[-1], self._offspring_size)

        # assert user return value
        assert type(parent_selections) == list
        assert all(type(s) == list for s in parent_selections)
        assert all(
            [
                all(self._is_tuple_individual_evaluation(p) for p in s)
                for s in parent_selections
            ]
        )

        # ignore user returned evaluation.
        # was only there to make it more convenient for the user
        parent_selections_only_individuals = [
            [p[0] for p in s] for s in parent_selections
        ]

        # let user create offspring
        offspring = [
            self._mutate(self._crossover(selection))
            for selection in parent_selections_only_individuals
        ]

        # assert user return value
        assert type(offspring) == list
        assert len(offspring) == self._offspring_size
        assert all(type(o) == self._individual_type for o in offspring)

        # let user evaluate offspring
        evaluation = await self._evaluate_generation(offspring)

        # assert user return value
        assert type(evaluation) == list
        assert len(evaluation) == len(offspring)
        assert all(type(e) == self._evaluation_type for e in evaluation)

        # combine individuals and evaluation
        evaluated_individuals = list(zip(offspring, evaluation))

        # let user select survivors between old and new individuals
        survivors = self._select_survivors(
            evaluated_individuals, self._generations[-1], self._population_size
        )

        # assert user return type
        assert type(survivors) == list
        assert len(survivors) == self._population_size
        assert all(self._is_tuple_individual_evaluation(s) for s in survivors)

        # set survivors as the next generation
        self._generations.append(survivors)

        # let user decide if optimizer must continue with another generation
        must_continue = self._must_do_next_gen()

        # assert user return type
        assert type(must_continue) == bool

        return must_continue

    def _is_tuple_individual_evaluation(self, item: Any) -> bool:
        """
        Check if type is Tuple[Individual, Evaluation]
        """

        return (
            type(item) == tuple
            and len(item) == 2
            and type(item[0]) == self._individual_type
            and type(item[1]) == self._evaluation_type
        )

    @property
    def generation_index(self) -> int:
        """
        Get the current generation.
        The initial generation is numbered 0.
        """

        return len(self._generations) - 1

    @property
    def generations(self) -> List[List[Tuple[Individual, Evaluation]]]:
        """
        Get all generations until now.
        """

        return self._generations
