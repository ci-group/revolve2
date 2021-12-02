from __future__ import annotations

import logging
import pickle
from abc import ABC, abstractmethod
from random import Random
from typing import Any, Generic, List, Optional, Type, TypeVar

from asyncinit import asyncinit
from revolve2.core.database import Database, Path
from revolve2.core.database.view import DictView

from .individual import Individual

Genotype = TypeVar("Genotype")
Evaluation = TypeVar("Evaluation")


@asyncinit
class EvolutionaryOptimizer(ABC, Generic[Genotype, Evaluation]):
    __database: Database
    __dbbranch: Path

    _rng: Random

    # next id to give to a new individual
    __next_id: int

    # Types of genotype and evaluation are stored as soon as they are available.
    # Used to type check the return values of user functions.
    __genotype_type: Type
    __evaluation_type: Optional[Type]

    __population_size: int
    __offspring_size: int

    __generation_index: Optional[int]
    __last_generation: Optional[List[Individual[Genotype, Evaluation]]]
    __initial_population: Optional[List[Genotype]]

    async def __init__(
        self,
        database: Database,
        dbbranch: Path,
        random: Random,
        population_size: int,
        offspring_size: int,
        initial_population: List[Genotype],
        initial_evaluation: Optional[List[Evaluation]],
    ):
        self.__database = database
        self.__dbbranch = dbbranch

        self._rng = random

        self.__next_id = 0

        logging.info("Attempting to load checkpoint..")
        if await self.load_checkpoint():
            logging.info(
                f"Checkpoint found. Last complete generation was {self.__generation_index}."
            )
        else:
            logging.info(
                f"No checkpoint has been made yet or if it has, database incompatible or corrupted. Starting with generation 0."
            )
            assert type(population_size) == int
            self.__population_size = population_size

            assert type(offspring_size) == int
            self.__offspring_size = offspring_size

            assert type(initial_population) == list
            assert len(initial_population) == self.__population_size
            assert len(initial_population) >= 1

            # save genotype type so we can type check things later
            self.__genotype_type = type(initial_population[0])

            if initial_evaluation is not None:
                assert type(initial_evaluation) == list
                assert len(initial_evaluation) == self.__population_size
                self.__last_generation = [
                    Individual[Genotype, Evaluation](
                        self._get_next_id(), genotype, evaluation
                    )
                    for genotype, evaluation in zip(
                        initial_population, initial_evaluation
                    )
                ]
                self.__generation_index = 0
                self.__initial_population = None

                # save evaluation type so we can type check things later
                self.__evaluation_type = type(initial_evaluation[0])
            else:
                self.__initial_population = initial_population
                self.__last_generation = None
                self.__generation_index = None

                # set evaluation type to None
                # we will set it when evaluating the first generation later
                self.__evaluation_type = None

    @abstractmethod
    async def _evaluate_generation(self, genotypes: List[Genotype]) -> List[Evaluation]:
        """
        Evaluate a genotype.

        :param genotypes: The genotypes to evaluate. Must not be altered.
        :return: The evaluation result.
        """

    @abstractmethod
    def _select_parents(
        self, generation: List[Individual[Genotype, Evaluation]], num_parents: int
    ) -> List[List[Individual[Genotype, Evaluation]]]:
        """
        Select groups of parents that will create offspring.

        :param population: The generation to select sets of parents from. Must not be altered.
        :return: The selected sets of parents.
        """

    def _select_survivors(
        self,
        old_individuals: List[Individual[Genotype, Evaluation]],
        new_individuals: List[Individual[Genotype, Evaluation]],
        num_survivors: int,
    ) -> List[Individual[Genotype, Evaluation]]:
        """
        Select survivors from a group of individuals. These will form the next generation.

        :param individuals: The individuals to choose from.
        :param num_survivors: How many individuals should be selected.
        :returns: The selected individuals.
        """

    @abstractmethod
    def _crossover(self, parents: List[Genotype]) -> Genotype:
        """
        Combine a set of genotypes into a new genotype.

        :param parents: The set of genotypes to combine. Must not be altered.
        :return: The new genotype.
        """

    @abstractmethod
    def _mutate(self, genotype: Genotype) -> Genotype:
        """
        Apply mutation to an genotype to create a new genotype.

        :param genotype: The original genotype. Must not be altered.
        :return: The new genotype.
        """

    @abstractmethod
    def _must_do_next_gen(self) -> bool:
        """
        Decide if the optimizer must do another generation.
        :returns: True if it must.
        """

    async def run(self) -> None:
        if self.__initial_population is not None:
            # let user evaluate. use unsafe version because we don't know evaluation type yet.
            evaluation = await self._evaluate_generation(self.__initial_population)

            # assert user return value
            assert type(evaluation) == list
            assert len(evaluation) == len(self.__initial_population)
            assert all(type(e) == type(evaluation[0]) for e in evaluation)

            # save evaluation type so we can type check things later
            self.__evaluation_type = type(evaluation[0])

            # combine provided genotypes and new evaluation to create the first generation
            self.__last_generation = [
                Individual[Genotype, Evaluation](
                    self._get_next_id(), genotype, evaluation
                )
                for genotype, evaluation in zip(self.__initial_population, evaluation)
            ]
            self.__generation_index = 0
            self.__initial_population = None

            await self._init_checkpoints()
            await self._save_checkpoint(self.__last_generation)

        while self._safe_must_do_next_gen():
            # let user select parents
            parent_selections = self._safe_select_parents(
                self.__last_generation, self.__offspring_size
            )

            # ignore user returned evaluation.
            # was only there to make it more convenient for the user
            parent_selections_only_individuals = [
                [p.genotype for p in s] for s in parent_selections
            ]

            # let user create offspring
            offspring = [
                self._safe_mutate(self._safe_crossover(selection))
                for selection in parent_selections_only_individuals
            ]

            # let user evaluate offspring
            evaluation = await self._safe_evaluate_generation(offspring)

            # combine to create list of individuals
            new_individuals = [
                Individual[Genotype, Evaluation](
                    self._get_next_id(), genotype, evaluation
                )
                for genotype, evaluation in zip(offspring, evaluation)
            ]

            # let user select survivors between old and new individuals
            survivors = self._safe_select_survivors(
                self.__last_generation, new_individuals, self.__population_size
            )

            # set survivors as the next generation
            self.__last_generation = survivors
            self.__generation_index += 1

            await self._save_checkpoint(new_individuals)

    def _get_next_id(self) -> int:
        next_id = self.__next_id
        self.__next_id += 1
        return next_id

    async def _init_checkpoints(self) -> None:
        """
        Initialize the checkpoint database.
        Saves all settings and sets up the generations list.
        """
        self.__database.begin_transaction()

        root = DictView(self.__database, self.__dbbranch)
        root.clear()

        root.insert(".rng").make_none()

        root.insert(".genotype_type").bytes = pickle.dumps(self.__genotype_type)
        root.insert(".evaluation_type").bytes = pickle.dumps(self.__evaluation_type)
        root.insert("population_size").int = self.__population_size
        root.insert("offspring_size").int = self.__offspring_size

        generations = root.insert("generations").list
        generations.clear()

        individuals = root.insert("individuals").list
        individuals.clear()

        self.__database.commit_transaction()

    async def _save_checkpoint(
        self, new_individuals: List[Individual[Genotype, Evaluation]]
    ) -> None:
        """
        Saves current random object and append the last generation to the checkpoint database.
        """

        self.__database.begin_transaction()

        root = DictView(self.__database, self.__dbbranch)

        root[".rng"].bytes = pickle.dumps(self._rng.getstate())

        generation = root["generations"].list.append().list
        generation.clear()
        for individual in self.__last_generation:
            generation.append().int = individual.id

        individuals = root["individuals"].list
        for individual in new_individuals:
            individuals.append().bytes = pickle.dumps(individual)

        self.__database.commit_transaction()

    async def load_checkpoint(self) -> bool:
        """
        Deserialize from the database.
        Can leave this class partially initialized if load unsuccessful.

        :returns: True if checkpoint could be loaded and everything is initialized from the database.
        """

        try:
            root = DictView(self.__database, self.__dbbranch)

            self._rng.setstate(pickle.loads(root[".rng"].bytes))

            self.__genotype_type = pickle.loads(root[".genotype_type"].bytes)
            self.__evaluation_type = pickle.loads(root[".evaluation_type"].bytes)
            self.__population_size = root["population_size"].int
            self.__offspring_size = root["offspring_size"].int

            generations = root["generations"].list
            individual_ids = [individual.int for individual in generations[-1].list]
            individuals_list = root["individuals"].list
            individuals = [
                pickle.loads(individuals_list[id].bytes) for id in individual_ids
            ]
            if not all([type(individual) == Individual for individual in individuals]):
                return False
            self.__last_generation = individuals
            self.__next_id = len(individuals_list)
            self.__generation_index = len(generations) - 1  # first generation is 0
        except (IndexError, pickle.PickleError):
            return False

        self.__initial_population = None

        return True

    def _safe_select_parents(
        self, generation: List[Individual[Genotype, Evaluation]], num_parents: int
    ) -> List[List[Individual[Genotype, Evaluation]]]:
        parent_selections = self._select_parents(generation, num_parents)
        assert type(parent_selections) == list
        assert all(type(s) == list for s in parent_selections)
        assert all([all(type(p) == Individual for p in s) for s in parent_selections])
        return parent_selections

    def _safe_crossover(self, parents: List[Genotype]) -> Genotype:
        genotype = self._crossover(parents)
        assert type(genotype) == self.__genotype_type
        return genotype

    def _safe_mutate(self, genotype: Genotype) -> Genotype:
        genotype = self._mutate(genotype)
        assert type(genotype) == self.__genotype_type
        return genotype

    async def _safe_evaluate_generation(
        self, genotypes: List[Genotype]
    ) -> List[Evaluation]:
        evaluations = await self._evaluate_generation(genotypes)
        assert type(evaluations) == list
        assert len(evaluations) == len(genotypes)
        assert all(type(e) == self.__evaluation_type for e in evaluations)
        return evaluations

    def _safe_select_survivors(
        self,
        old_individuals: List[Individual[Genotype, Evaluation]],
        new_individuals: List[Individual[Genotype, Evaluation]],
        num_survivors: int,
    ) -> List[Individual[Genotype, Evaluation]]:
        survivors = self._select_survivors(
            old_individuals, new_individuals, num_survivors
        )
        assert type(survivors) == list
        assert len(survivors) == self.__population_size
        assert all(type(s) == Individual for s in survivors)
        return survivors

    def _safe_must_do_next_gen(self) -> bool:
        must_do = self._must_do_next_gen()
        assert type(must_do) == bool
        return must_do

    @property
    def population_size(self) -> int:
        return self.__population_size

    @property
    def offspring_size(self) -> int:
        return self.__offspring_size

    @property
    def generation_index(self) -> int:
        """
        Get the current generation.
        The initial generation is numbered 0.
        """

        return self.__generation_index

    @property
    def last_generation(self) -> List[Individual[Genotype, Evaluation]]:
        """
        Get the last generation.
        """

        return self.__last_generation
