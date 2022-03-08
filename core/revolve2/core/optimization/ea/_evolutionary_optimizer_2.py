from __future__ import annotations

import logging
import pickle
from abc import ABC, abstractmethod
from random import Random
from typing import Generic, List, Optional, Type, TypeVar, Union

from revolve2.core.database import Database
from revolve2.core.database import List as DbList
from revolve2.core.database import (
    Node,
    Transaction,
    Uninitialized,
    dynamic_cast_bytes,
    dynamic_cast_dblist,
    dynamic_cast_int,
    dynamic_cast_dict,
    dynamic_cast_list,
    dynamic_cast_node,
    dynamic_cast_static_data,
)
from revolve2.serialization import Serializable, SerializeError, StaticData
from revolve2.core.optimization import Optimizer

from ._individual import Individual

Genotype = TypeVar("Genotype", bound=Union[Serializable, StaticData])
Fitness = TypeVar("Fitness", bound=Union[Serializable, StaticData])


class EvolutionaryOptimizer(Optimizer, ABC, Generic[Genotype, Fitness]):
    __database: Database
    __db_node: Node

    _rng: Random

    # next id to give to a new individual
    __next_id: int

    # Types of genotype and fitness are stored as soon as they are available.
    # Used to type check the return values of user functions.
    __genotype_type: Type[Genotype]
    __fitness_type: Optional[type]

    __population_size: int
    __offspring_size: int

    __generation_index: Optional[int]
    __last_generation: Optional[List[Individual[Genotype, Fitness]]]
    __initial_population: Optional[List[Genotype]]

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

    @abstractmethod
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

    async def ainit(
        self,
        random: Random,
        population_size: int,
        offspring_size: int,
        initial_population: List[Genotype],
        initial_fitness: Optional[List[Fitness]],
    ) -> None:
        self._rng = random

        self.__next_id = 0

        assert isinstance(population_size, int)
        self.__population_size = population_size

        assert isinstance(offspring_size, int)
        self.__offspring_size = offspring_size

        assert isinstance(initial_population, list)
        assert len(initial_population) == self.__population_size
        assert len(initial_population) >= 1

        # save genotype type so we can type check things later
        self.__genotype_type = type(initial_population[0])

        if initial_fitness is not None:
            assert isinstance(initial_fitness, list)
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

            # same with these database nodes
            self.__db_rng_after_generation = None
            self.__db_generations = None
            self.__db_individuals = None

    async def ainit_from_database(
        self, database: Database, db_node: Node, random: Random
    ) -> bool:
        self.__database = database
        self.__db_node = db_node
        self._rng = random

        with self.__database.begin_transaction() as txn:
            root = self.__db_node.get_db_data(txn)

            try:
                if isinstance(root, Uninitialized):
                    return False
                else:
                    if not isinstance(root, dict):
                        raise SerializeError()
                    self.__db_ea = dynamic_cast_node(root["ea"])
                    self.__db_evaluations = dynamic_cast_dblist(
                        dynamic_cast_node(root["evaluations"]).get_db_data(txn)
                    )

                    ea = dynamic_cast_dict(self.__db_ea.get_db_data(txn))

                    self.__db_rng_after_generation = dynamic_cast_dblist(
                        dynamic_cast_node(ea["rng_after_generation"]).get_db_data(txn)
                    )
                    self.__population_size = dynamic_cast_int(ea["population_size"])
                    self.__offspring_size = dynamic_cast_int(ea["offspring_size"])
                    self.__db_generations = dynamic_cast_dblist(
                        dynamic_cast_node(ea["generations"]).get_db_data(txn)
                    )
                    self.__db_individuals = dynamic_cast_dblist(
                        dynamic_cast_node(ea["individuals"]).get_db_data(txn)
                    )

                    self.__genotype_type = pickle.loads(ea["genotype_type"])
                    self.__fitness_type = pickle.loads(ea["fitness_type"])

                    self.__generation_index = (
                        self.__db_generations.len(txn) - 1
                    )  # first generation is 0

                    individual_ids = dynamic_cast_list(
                        self.__db_generations.get(
                            txn, self.__generation_index
                        ).get_db_data(txn)
                    )
                    self.__last_generation = [
                        Individual.deserialize(
                            dynamic_cast_static_data(
                                self.__db_individuals.get(txn, id).get_db_data(txn)
                            )
                        )
                        for id in individual_ids
                    ]
                    self.__next_id = self.__db_individuals.len(txn)

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

            logging.info("Finished generation 0.")

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

            self.__db_ea.set_db_data(
                txn,
                {
                    "rng_after_generation": self.__db_rng_after_generation,
                    "genotype_type": pickle.dumps(self.__genotype_type),
                    "fitness_type": pickle.dumps(self.__fitness_type),
                    "population_size": self.__population_size,
                    "offspring_size": self.__offspring_size,
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
                    self.__db_ea = Node()
                    self.__db_evaluations = DbList()
                    self.__db_node.set_db_data(
                        txn, {"evaluations": self.__db_evaluations, "ea": self.__db_ea}
                    )
                    return False
                else:
                    if not isinstance(root, dict):
                        raise SerializeError()
                    self.__db_ea = root["ea"]
                    if not isinstance(self.__db_ea, Node):
                        raise SerializeError()
                    self.__db_evaluations = dynamic_cast_dblist(
                        dynamic_cast_node(root["evaluations"]).get_db_data(txn)
                    )
                    if not isinstance(self.__db_evaluations, DbList):
                        raise SerializeError()

                    ea = self.__db_ea.get_db_data(txn)
                    if isinstance(ea, Uninitialized):
                        return False
                    elif not isinstance(ea, dict):
                        raise SerializeError()

                    self.__db_rng_after_generation = dynamic_cast_dblist(
                        dynamic_cast_node(ea["rng_after_generation"]).get_db_data(txn)
                    )
                    if not isinstance(self.__db_rng_after_generation, DbList):
                        raise SerializeError()
                    self.__population_size = ea["population_size"]
                    if not isinstance(self.__population_size, int):
                        raise SerializeError()
                    self.__offspring_size = ea["offspring_size"]
                    if not isinstance(self.__offspring_size, int):
                        raise SerializeError()
                    self.__db_generations = dynamic_cast_dblist(
                        dynamic_cast_node(ea["generations"]).get_db_data(txn)
                    )
                    if not isinstance(self.__db_generations, DbList):
                        raise SerializeError()
                    self.__db_individuals = dynamic_cast_dblist(
                        dynamic_cast_node(ea["individuals"]).get_db_data(txn)
                    )
                    if not isinstance(self.__db_individuals, DbList):
                        raise SerializeError()

                    self.__genotype_type = pickle.loads(ea["genotype_type"])
                    self.__fitness_type = pickle.loads(ea["fitness_type"])

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
