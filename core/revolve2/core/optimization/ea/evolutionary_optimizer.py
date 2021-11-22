from __future__ import annotations

import pickle
from abc import ABC, abstractmethod
from typing import Any, Generic, List, Optional, Tuple, Type, TypeVar

from asyncinit import asyncinit
from revolve2.core import database
from revolve2.core.database import Database, Path
from revolve2.core.database.view import BytesView, DictView, IntView, ListView
from revolve2.core.database.view.bytes_view import BytesView

Individual = TypeVar("Individual")
Evaluation = TypeVar("Evaluation")


@asyncinit
class EvolutionaryOptimizer(ABC, Generic[Individual, Evaluation]):
    _database: Database
    _dbbranch: Path

    # Types of individual and evaluation are stored as soon as they are available.
    # Used to type check the return values of user functions.
    _individual_type: Type
    _evaluation_type: Optional[Type]

    _population_size: int
    _offspring_size: int

    _generations: List[List[Tuple[Individual, Evaluation]]]
    _initial_population: Optional[List[Individual]]

    async def __init__(
        self,
        database: Database,
        dbbranch: Path,
        population_size: int,
        offspring_size: int,
        initial_population: List[Individual],
        initial_evaluation: Optional[List[Evaluation]],
    ):
        self._database = database
        self._dbbranch = dbbranch

        if not await self.load_checkpoint():
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
                self._initial_population = None

                # save evaluation type so we can type check things later
                self._evaluation_type = type(initial_evaluation[0])
            else:
                self._initial_population = initial_population
                self._generations = []

                # set evaluation type to None
                # we will set it when evaluating the first generation later
                self._evaluation_type = None

            await self.save_checkpoint()
            await self.load_checkpoint()

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
        """self._database = database
        self._dbbranch = dbbranchneration.

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

    async def run(self) -> None:
        if self._initial_population is not None:
            # let user evaluate. use unsafe version because we don't know evaluation type yet.
            evaluation = await self._evaluate_generation(self._initial_population)

            # assert user return value
            assert type(evaluation) == list
            assert len(evaluation) == len(self._initial_population)
            assert all(type(e) == type(evaluation[0]) for e in evaluation)

            # save evaluation type so we can type check things later
            self._evaluation_type = type(evaluation[0])

            # combine provided individuals and new evaluation to create the first generation
            self._generations.append(list(zip(self._initial_population, evaluation)))
            self._initial_population = None

            await self.save_checkpoint()
            await self.load_checkpoint()

        while self._safe_must_do_next_gen():
            # let user select parents
            parent_selections = self._safe_select_parents(
                self._generations[-1], self._offspring_size
            )

            # ignore user returned evaluation.
            # was only there to make it more convenient for the user
            parent_selections_only_individuals = [
                [p[0] for p in s] for s in parent_selections
            ]

            # let user create offspring
            offspring = [
                self._safe_mutate(self._safe_crossover(selection))
                for selection in parent_selections_only_individuals
            ]

            # let user evaluate offspring
            evaluation = await self._safe_evaluate_generation(offspring)

            # combine individuals and evaluation
            evaluated_individuals = list(zip(offspring, evaluation))

            # let user select survivors between old and new individuals
            survivors = self._safe_select_survivors(
                evaluated_individuals, self._generations[-1], self._population_size
            )

            # set survivors as the next generation
            self._generations.append(survivors)

            await self.save_checkpoint()
            await self.load_checkpoint()

    async def save_checkpoint(self) -> None:
        """
        Serialize process to the database.
        """

        self._database.begin_transaction()

        root = DictView(self._database, self._dbbranch)
        root.clear()

        root.insert("_individual_type").bytes.val = pickle.dumps(self._individual_type)
        root.insert("_evaluation_type").bytes.val = pickle.dumps(self._evaluation_type)
        root.insert("population_size").int.val = pickle.dumps(self._population_size)
        root.insert("offspring_size").int.val = pickle.dumps(self._offspring_size)

        generations = root.insert("generations").list
        generations.clear()
        for generation in self._generations:
            db_gen = generations.append().list
            db_gen.clear()
            for individual in generation:
                db_gen.append().bytes.val = pickle.dumps(individual)

        initial_population = root.insert("_initial_population")
        if self._initial_population is None:
            initial_population.make_none()
        else:
            initial_population.list.clear()
            for individual in self._initial_population:
                initial_population.list.append().bytes.val = pickle.dumps(individual)

        self._database.commit_transaction()

    async def load_checkpoint(self) -> bool:
        """
        Deserialize from the database.
        Can leave this class partially initialized if load unsuccessful.

        :returns: True if checkpoint could be loaded and everything is initialized from the database.
        """

        try:
            root = DictView(self._database, self._dbbranch)

            self._individual_type = pickle.loads(root["_individual_type"].bytes.val)
            self._evaluation_type = pickle.loads(root["_evaluation_type"].bytes.val)
            self._population_size = root["population_size"].int.val
            self._offspring_size = root["offspring_size"].int.val

            for generation in root["generations"].list:
                self._generations.append([])
                for individual in generation.list:
                    individual_loaded = pickle.loads(individual.bytes.val)
                    if not self._is_tuple_individual_evaluation(individual_loaded):
                        return False
                    self._generations[-1].append(individual_loaded)

            initial_population = root["_initial_population"]
            if initial_population.is_list():
                for individual in initial_population.list:
                    individual_loaded = pickle.loads(individual.bytes.val)
                    if not self._is_tuple_individual_evaluation(individual_loaded):
                        return False
                    self._generations[-1].append(individual_loaded)
            elif initial_population.is_none():
                self._initial_population = None
            else:
                return False
        except (RuntimeError, pickle.PickleError):
            return False

        return True

    def _safe_select_parents(
        self, generation: List[Tuple[Individual, Evaluation]], num_parents: int
    ) -> List[List[Tuple[Individual, Evaluation]]]:
        parent_selections = self._select_parents(
            self._generations[-1], self._offspring_size
        )
        assert type(parent_selections) == list
        assert all(type(s) == list for s in parent_selections)
        assert all(
            [
                all(self._is_tuple_individual_evaluation(p) for p in s)
                for s in parent_selections
            ]
        )
        return parent_selections

    def _safe_crossover(self, parents: List[Individual]) -> Individual:
        individual = self._crossover(parents)
        assert type(individual) == self._individual_type
        return individual

    def _safe_mutate(self, individual: Individual) -> Individual:
        individual = self._mutate(individual)
        assert type(individual) == self._individual_type
        return individual

    async def _safe_evaluate_generation(
        self, individuals: List[Individual]
    ) -> List[Evaluation]:
        evaluations = await self._evaluate_generation(individuals)
        assert type(evaluations) == list
        assert len(evaluations) == len(individuals)
        assert all(type(e) == self._evaluation_type for e in evaluations)
        return evaluations

    def _safe_select_survivors(
        self,
        old_individuals: List[Tuple[Individual, Evaluation]],
        new_individuals: List[Tuple[Individual, Evaluation]],
        num_survivors: int,
    ) -> List[Tuple[Individual, Evaluation]]:
        survivors = self._select_survivors(
            old_individuals, new_individuals, num_survivors
        )
        assert type(survivors) == list
        assert len(survivors) == self._population_size
        assert all(self._is_tuple_individual_evaluation(s) for s in survivors)
        return survivors

    def _safe_must_do_next_gen(self) -> bool:
        must_do = self._must_do_next_gen()
        assert type(must_do) == bool
        return must_do

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
