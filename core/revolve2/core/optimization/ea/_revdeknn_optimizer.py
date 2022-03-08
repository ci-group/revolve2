from __future__ import annotations

import logging
import pickle
from abc import ABC, abstractmethod
from random import Random
from typing import List, Optional, Any

from revolve2.core.database import Database
from revolve2.core.database import List as DbList
from revolve2.core.database import (
    Node,
    Transaction,
    Uninitialized,
    dynamic_cast_bytes,
    dynamic_cast_dblist,
    dynamic_cast_node,
    dynamic_cast_static_data,
)
from revolve2.serialization import SerializeError
import numpy as np
import nptyping as npt

from ._individual import Individual


class RevdeknnOptimizer(ABC):
    Genotype = npt.NDArray[Any, np.float_]
    Fitness = float

    __database: Database
    __db_node: Node

    _rng: Random

    # next id to give to a new individual
    __next_id: int

    __population_size: int
    __offspring_size: int

    __generation_index: Optional[int]
    __last_generation: Optional[List[Individual[Genotype, Fitness]]]
    __initial_population: Optional[List[Genotype]]

    __db_optimizer: Node  # All db data that is not evaluations.
    __db_evaluations: DbList  # A node for every generation, provided to the user to store their evaluation.

    __db_rng_after_generation: Optional[
        DbList
    ]  # The Random object state directly after the generation matching its index.
    __db_generations: Optional[DbList]  # Lists of invidiual ids
    __db_individuals: Optional[DbList]  # All individuals, indexed by id

    __transformation: npt.NDArray[1, 3, 3, np.float_]

    def __init__(self) -> None:
        raise ValueError(
            "Do not call this function. Call asyncinit instead. Inherit this class and implement __init__ to avoid this error."
        )

    async def asyncinit(
        self,
        database: Database,
        db_node: Node,
        random: Random,
        initial_population: npt.NDArray[Any, Any, np.float_],
        initial_fitness: Optional[npt.NDArray[Any, np.float_]],
    ) -> None:
        """
        :initial_population: n x d, with n the number of individuals in a population and d the dimension of a genotype.
        :initial_fitness: n x 1, with n the number of individuals in a population.
        """

        self.__database = database
        self.__db_node = db_node

        self._rng = random

        self.__next_id = 0

        logging.info("Attempting to load checkpoint..")
        if await self._load_checkpoint_or_init():
            logging.info(
                f"Checkpoint found. Last complete generation was {self.__generation_index}."
            )
        else:
            logging.info(
                f"No checkpoint has been made yet. Starting with generation 0."
            )

            assert isinstance(initial_population, np.array)
            assert initial_population.ndim == 2
            assert initial_population.shape()[0] >= 1
            assert initial_population.shape()[1] >= 1
            assert initial_population.dtype == np.float_

            if initial_fitness is not None:
                assert isinstance(initial_fitness, np.array)
                assert initial_fitness.ndim == 1
                assert initial_fitness.shape()[0] == initial_population.shape()[0]
                assert initial_fitness.dtype == np.float_

                self.__last_generation = [
                    Individual[self.Genotype, self.Fitness](
                        self._get_next_id(), genotype, fitness, None
                    )
                    for genotype, fitness in zip(initial_population, initial_fitness)
                ]
                self.__generation_index = 0
                self.__initial_population = None

                await self._save_zeroth_generation(self.__last_generation)
            else:
                self.__initial_population = initial_population
                self.__last_generation = None
                self.__generation_index = None

                # we will set these after evaluating the first generation later
                self.__db_rng_after_generation = None
                self.__db_generations = None
                self.__db_individuals = None

    @abstractmethod
    async def _evaluate_generation(
        self, genotypes: List[Genotype], database: Database, db_node: Node
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
                Individual[self.Genotype, self.Fitness](
                    self._get_next_id(), genotype, fitness, None
                )
                for genotype, fitness in zip(self.__initial_population, fitness)
            ]
            self.__generation_index = 0
            self.__initial_population = None

            await self._save_zeroth_generation(self.__last_generation)

            logging.info("Finished generation 0.")

        while self._safe_must_do_next_gen():
            assert self.__generation_index is not None
            assert self.__last_generation is not None

            offspring = 3  # TODO

            # let user evaluate offspring
            db_evaluation_branch = await self._prepare_db_evaluation()
            fitness = await self._safe_evaluate_generation(
                offspring, self.__database, db_evaluation_branch
            )

            # combine to create list of individuals
            new_individuals = [
                Individual[self.Genotype, self.Fitness](
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

            logging.info(f"Finished generation {self.__generation_index}.")

    def _get_next_id(self) -> int:
        next_id = self.__next_id
        self.__next_id += 1
        return next_id

    async def _save_zeroth_generation(
        self, new_individuals: List[Individual[Genotype, Fitness]]
    ) -> None:
        """
        Save parameters to the database and do `_save_generation` for the initial population.
        """

        with self.__database.begin_transaction() as txn:
            self.__db_rng_after_generation = DbList()
            self.__db_generations = DbList()
            self.__db_individuals = DbList()

            self.__db_optimizer.set_db_data(
                txn,
                {
                    "rng_after_generation": self.__db_rng_after_generation,
                    "generations": self.__db_generations,
                    "individuals": self.__db_individuals,
                },
            )

            await self._save_generation_notransaction(txn, new_individuals)

    async def _save_generation(
        self, new_individuals: List[Individual[Genotype, Fitness]]
    ) -> None:
        """
        Append the last generation and accompanying random object to the checkpoint database.

        :param new_individuals: Individuals borns during last generation
        """

        with self.__database.begin_transaction() as txn:
            await self._save_generation_notransaction(txn, new_individuals)

    async def _save_generation_notransaction(
        self, txn: Transaction, new_individuals: List[Individual[Genotype, Fitness]]
    ) -> None:
        assert self.__last_generation is not None
        assert self.__db_rng_after_generation is not None
        assert self.__db_generations is not None
        assert self.__db_individuals is not None

        logging.debug("Beginning saving generation..")

        self.__db_rng_after_generation.append(txn).set_db_data(
            txn, pickle.dumps(self._rng.getstate())
        )
        self.__db_generations.append(txn).set_db_data(
            txn, [individual.id for individual in self.__last_generation]
        )
        try:
            for individual in new_individuals:
                self.__db_individuals.append(txn).set_db_data(
                    txn, individual.serialize()
                )
        except SerializeError as err:
            raise SerializeError("Failed to serialize individual.") from err

        logging.debug("Finished saving generation.")

    async def _load_checkpoint_or_init(self) -> bool:
        """
        Deserialize from the database. If no checkpoint was found, the basic database structure is created.

        :returns: True if checkpoint could be loaded and everything is initialized from the database.
                  False if there was no checkpoint.
        :raises SerializeError: If the database is in an incompatible state and the optimizer cannot continue.
                                Can leave this class in a partially deserialized state.
        """

        with self.__database.begin_transaction() as txn:
            root = self.__db_node.get_db_data(txn)

            try:
                if isinstance(root, Uninitialized):
                    self.__db_optimizer = Node()
                    self.__db_evaluations = DbList()
                    self.__db_node.set_db_data(
                        txn,
                        {
                            "evaluations": self.__db_evaluations,
                            "optimizer": self.__db_optimizer,
                        },
                    )
                    return False
                else:
                    if not isinstance(root, dict):
                        raise SerializeError()
                    self.__db_optimizer = root["optimizer"]
                    if not isinstance(self.__db_optimizer, Node):
                        raise SerializeError()
                    self.__db_evaluations = dynamic_cast_dblist(
                        dynamic_cast_node(root["evaluations"]).get_db_data(txn)
                    )
                    if not isinstance(self.__db_evaluations, DbList):
                        raise SerializeError()

                    optimizer = self.__db_optimizer.get_db_data(txn)
                    if isinstance(optimizer, Uninitialized):
                        return False
                    elif not isinstance(optimizer, dict):
                        raise SerializeError()

                    self.__db_rng_after_generation = dynamic_cast_dblist(
                        dynamic_cast_node(
                            optimizer["rng_after_generation"]
                        ).get_db_data(txn)
                    )
                    if not isinstance(self.__db_rng_after_generation, DbList):
                        raise SerializeError()
                    self.__db_generations = dynamic_cast_dblist(
                        dynamic_cast_node(optimizer["generations"]).get_db_data(txn)
                    )
                    if not isinstance(self.__db_generations, DbList):
                        raise SerializeError()
                    self.__db_individuals = dynamic_cast_dblist(
                        dynamic_cast_node(optimizer["individuals"]).get_db_data(txn)
                    )
                    if not isinstance(self.__db_individuals, DbList):
                        raise SerializeError()

                    self.__generation_index = (
                        self.__db_generations.len(txn) - 1
                    )  # first generation is 0

                    individual_ids = self.__db_generations.get(
                        txn, self.__generation_index
                    ).get_db_data(txn)
                    if not isinstance(individual_ids, list):
                        raise SerializeError()
                    self.__last_generation = [
                        Individual.deserialize(
                            dynamic_cast_static_data(
                                self.__db_individuals.get(txn, id).get_db_data(txn)
                            )
                        )
                        for id in individual_ids
                    ]
                    self.__next_id = self.__db_individuals.len(txn)

                    x = self.__db_rng_after_generation.get(
                        txn, self.__generation_index
                    ).get_db_data(txn)

                    self._rng.setstate(
                        pickle.loads(
                            dynamic_cast_bytes(
                                self.__db_rng_after_generation.get(
                                    txn, self.__generation_index
                                ).get_db_data(txn)
                            )
                        )
                    )

                    self.__initial_population = None

                    return True

            except (SerializeError, pickle.PickleError, KeyError, TypeError) as err:
                raise SerializeError(
                    "Database in state incompatible with this code. Remove database before trying again."
                ) from err

    async def _prepare_db_evaluation(self) -> Node:
        if self.__generation_index is None:
            generation = 0
        else:
            generation = self.__generation_index + 1

        with self.__database.begin_transaction() as txn:
            return self.__db_evaluations.get_or_append(txn, generation)

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
        self, genotypes: List[Genotype], database: Database, db_node: Node
    ) -> List[Fitness]:
        fitnesss = await self._evaluate_generation(genotypes, database, db_node)
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
