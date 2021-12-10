from __future__ import annotations

import logging
import pickle
from abc import ABC, abstractmethod
from random import Random
from typing import Any, Generic, List, Optional, Type, TypeVar, cast

from asyncinit import asyncinit
from revolve2.core.database import Database, DatabaseError, View
from revolve2.core.database.serialize import Serializable
from revolve2.core.database.serialize.serialize_error import SerializeError

from .individual import Individual

Genotype = TypeVar("Genotype", bound=Serializable)
Fitness = TypeVar("Fitness", bound=Serializable)


@asyncinit
class EvolutionaryOptimizer(ABC, Generic[Genotype, Fitness]):
    __database: Database
    __dbview: View

    _rng: Random

    # next id to give to a new individual
    __next_id: int

    # Types of genotype and fitness are stored as soon as they are available.
    # Used to type check the return values of user functions.
    __genotype_type: Type
    __fitness_type: Optional[Type]

    __population_size: int
    __offspring_size: int

    __generation_index: Optional[int]
    __last_generation: Optional[List[Individual[Genotype, Fitness]]]
    __initial_population: Optional[List[Genotype]]

    async def __init__(
        self,
        database: Database,
        dbview: View,
        random: Random,
        population_size: int,
        offspring_size: int,
        initial_population: List[Genotype],
        initial_fitness: Optional[List[Fitness]],
    ):
        self.__database = database
        self.__dbview = dbview

        self._rng = random

        self.__next_id = 0

        logging.info("Attempting to load checkpoint..")
        if await self._load_checkpoint():
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

            if initial_fitness is not None:
                assert type(initial_fitness) == list
                assert len(initial_fitness) == self.__population_size
                self.__last_generation = [
                    Individual[Genotype, Fitness](
                        self._get_next_id(), genotype, fitness, None
                    )
                    for genotype, fitness in zip(initial_population, initial_fitness)
                ]
                self.__generation_index = 0
                self.__initial_population = None

                # save fitness type so we can type check things later
                self.__fitness_type = type(initial_fitness[0])

                await self._save_zeroth_generation(self.__last_generation)
            else:
                self.__initial_population = initial_population
                self.__last_generation = None
                self.__generation_index = None

                # set fitness type to None
                # we will set it when evaluating the first generation later
                self.__fitness_type = None

            await self._init_root()

    @abstractmethod
    async def _evaluate_generation(
        self, genotypes: List[Genotype], database: Database, dbview: View
    ) -> List[Fitness]:
        """
        Evaluate a genotype.

        :param genotypes: The genotypes to evaluate. Must not be altered.
        :return: The fitness result.
        """

    @abstractmethod
    def _select_parents(
        self, generation: List[Individual[Genotype, Fitness]], num_parents: int
    ) -> List[List[Individual[Genotype, Fitness]]]:
        """
        Select groups of parents that will create offspring.

        :param population: The generation to select sets of parents from. Must not be altered.
        :return: The selected sets of parents.
        """

    def _select_survivors(
        self,
        old_individuals: List[Individual[Genotype, Fitness]],
        new_individuals: List[Individual[Genotype, Fitness]],
        num_survivors: int,
    ) -> List[Individual[Genotype, Fitness]]:
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
            # let user evaluate. use unsafe version because we don't know fitness type yet.
            db_evaluation_branch = await self._prepare_db_evaluation()
            fitness = await self._evaluate_generation(
                self.__initial_population, self.__database, db_evaluation_branch
            )

            # assert user return value
            assert type(fitness) == list
            assert len(fitness) == len(self.__initial_population)
            assert all(type(e) == type(fitness[0]) for e in fitness)

            # save fitness type so we can type check things later
            self.__fitness_type = type(fitness[0])

            # combine provided genotypes and new fitness to create the first generation
            self.__last_generation = [
                Individual[Genotype, Fitness](
                    self._get_next_id(), genotype, fitness, None
                )
                for genotype, fitness in zip(self.__initial_population, fitness)
            ]
            self.__generation_index = 0
            self.__initial_population = None

            await self._save_zeroth_generation(self.__last_generation)

        while self._safe_must_do_next_gen():
            assert self.__generation_index is not None
            assert self.__last_generation is not None

            # let user select parents
            parent_selections = self._safe_select_parents(
                self.__last_generation, self.__offspring_size
            )

            # let user create offspring
            offspring = [
                self._safe_mutate(self._safe_crossover(genotype))
                for genotype in [[p.genotype for p in s] for s in parent_selections]
            ]

            # let user evaluate offspring
            db_evaluation_branch = await self._prepare_db_evaluation()
            fitness = await self._safe_evaluate_generation(
                offspring, self.__database, db_evaluation_branch
            )

            # combine to create list of individuals
            new_individuals = [
                Individual[Genotype, Fitness](
                    self._get_next_id(),
                    genotype,
                    fitness,
                    [parent.id for parent in parents],
                )
                for parents, genotype, fitness in zip(
                    parent_selections, offspring, fitness
                )
            ]

            # let user select survivors between old and new individuals
            survivors = self._safe_select_survivors(
                self.__last_generation, new_individuals, self.__population_size
            )

            # set survivors as the next generation
            self.__last_generation = survivors
            self.__generation_index += 1

            await self._save_generation(new_individuals)

    def _get_next_id(self) -> int:
        next_id = self.__next_id
        self.__next_id += 1
        return next_id

    async def _init_root(self) -> None:
        self.__database.begin_transaction()
        self.__dbview.data = {"evaluations": []}
        self.__database.commit_transaction()

    async def _save_zeroth_generation(
        self, new_individuals: List[Individual[Genotype, Fitness]]
    ) -> None:
        """
        Save parameters to the database and do `_save_generation` for the initial population.
        """

        self.__database.begin_transaction()

        self.__dbview[".rng_after_generation"] = []
        self.__dbview[".genotype_type"] = pickle.dumps(self.__genotype_type)
        self.__dbview[".fitness_type"] = pickle.dumps(self.__fitness_type)
        self.__dbview["population_size"] = self.__population_size
        self.__dbview["offspring_size"] = self.__offspring_size
        self.__dbview["generations"] = []
        self.__dbview["individuals"] = []

        await self._save_generation_notransaction(new_individuals)

        self.__database.commit_transaction()

    async def _save_generation(
        self, new_individuals: List[Individual[Genotype, Fitness]]
    ) -> None:
        """
        Append the last generation and accompanying random object to the checkpoint database.

        :param new_individuals: Individuals borns during last generation
        """

        self.__database.begin_transaction()
        await self._save_generation_notransaction(new_individuals)
        self.__database.commit_transaction()

    async def _save_generation_notransaction(
        self, new_individuals: List[Individual[Genotype, Fitness]]
    ):
        assert self.__last_generation is not None

        self.__dbview[".rng_after_generation"].append(
            pickle.dumps(self._rng.getstate())
        )
        self.__dbview["generations"].append(
            [individual.id for individual in self.__last_generation]
        )
        self.__dbview["individuals"].extend(
            [individual.serialize() for individual in new_individuals]
        )

    async def _load_checkpoint(self) -> bool:
        """
        Deserialize from the database.
        Can leave this class partially initialized if load unsuccessful.

        :returns: True if checkpoint could be loaded and everything is initialized from the database.
        """

        try:
            self.__genotype_type = pickle.loads(self.__dbview[".genotype_type"].data)
            self.__fitness_type = pickle.loads(self.__dbview[".fitness_type"].data)
            self.__population_size = self.__dbview["population_size"].data
            if type(self.__population_size) != int:
                raise SerializeError()
            self.__offspring_size = self.__dbview["offspring_size"].data
            if type(self.__offspring_size) != int:
                raise SerializeError()

            generations_view = self.__dbview["generations"]
            individual_ids = cast(List[int], generations_view[-1].data)
            individuals_view = self.__dbview["individuals"]
            individuals = [
                Individual.deserialize(individuals_view[id].data)
                for id in individual_ids
            ]
            self.__last_generation = individuals
            self.__next_id = len(individuals_view)
            self.__generation_index = len(generations_view) - 1  # first generation is 0

            self._rng.setstate(
                pickle.loads(
                    self.__dbview[".rng_after_generation"][self.__generation_index].data
                )
            )
        except (DatabaseError, pickle.PickleError, SerializeError):
            return False

        self.__initial_population = None

        return True

    async def _prepare_db_evaluation(self) -> View:
        if self.__generation_index is None:
            generation = 0
        else:
            generation = self.__generation_index + 1

        evaluations_view = self.__dbview["evaluations"]
        if len(evaluations_view) <= generation:
            evaluations_view.append(None)
        return evaluations_view[-1]

    def _safe_select_parents(
        self, generation: List[Individual[Genotype, Fitness]], num_parents: int
    ) -> List[List[Individual[Genotype, Fitness]]]:
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
        self, genotypes: List[Genotype], database: Database, dbview: View
    ) -> List[Fitness]:
        fitnesss = await self._evaluate_generation(genotypes, database, dbview)
        assert type(fitnesss) == list
        assert len(fitnesss) == len(genotypes)
        assert all(type(e) == self.__fitness_type for e in fitnesss)
        return fitnesss

    def _safe_select_survivors(
        self,
        old_individuals: List[Individual[Genotype, Fitness]],
        new_individuals: List[Individual[Genotype, Fitness]],
        num_survivors: int,
    ) -> List[Individual[Genotype, Fitness]]:
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
    def generation_index(self) -> Optional[int]:
        """
        Get the current generation.
        The initial generation is numbered 0.
        """

        return self.__generation_index

    @property
    def last_generation(self) -> Optional[List[Individual[Genotype, Fitness]]]:
        """
        Get the last generation.
        """

        return self.__last_generation
