from __future__ import annotations
from typing import Generic, TypeVar, Type, List, Optional, Tuple

from revolve2.core.optimization import Process, ProcessIdGen
from abc import abstractmethod
from revolve2.core.database import Database, IncompatibleError
from sqlalchemy.orm.exc import NoResultFound
from sqlalchemy.orm.exc import MultipleResultsFound
from sqlalchemy.future import select
from sqlalchemy.exc import OperationalError
from dataclasses import dataclass
from ._evolutionary_optimizer_schema import (
    DbEvolutionaryOptimizer,
    DbEvolutionaryOptimizerGeneration,
    DbEvolutionaryOptimizerIndividual,
    DbEvolutionaryOptimizerState,
    DbBase,
)

Child = TypeVar("Child")
Genotype = TypeVar("Genotype")  # TODO bounds
Fitness = TypeVar("Fitness")


class EvolutionaryOptimizer(Process[Child], Generic[Child, Genotype, Fitness]):
    @abstractmethod
    async def _evaluate_generation(
        self,
        genotypes: List[Genotype],
        database: Database,
        process_id: int,
        process_id_gen: ProcessIdGen,
    ) -> List[Fitness]:
        """
        Evaluate a genotype.

        :param genotypes: The genotypes to evaluate. Must not be altered.
        :return: The fitness result.
        """

    @abstractmethod
    def _select_parents(
        self,
        population: List[Genotype],
        fitnesses: List[Fitness],
        num_parent_groups: int,
    ) -> List[List[int]]:
        """
        Select groups of parents that will create offspring.

        :param population: The generation to select sets of parents from. Must not be altered.
        :return: The selected sets of parents, each integer representing a population index.
        """

    @abstractmethod
    def _select_survivors(
        self,
        old_individuals: List[Genotype],
        old_fitnesses: List[Fitness],
        new_individuals: List[Genotype],
        new_fitnesses: List[Fitness],
        num_survivors: int,
    ) -> Tuple(List[int], List[int]):
        """
        Select survivors from a group of individuals. These will form the next generation.

        :param individuals: The individuals to choose from.
        :param num_survivors: How many individuals should be selected.
        :return: Indices of the old survivors and indices of the new survivors.
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
        :return: True if it must.
        """

    @dataclass
    class __Individual(Generic[Genotype]):
        id: int
        genotype: Genotype
        parent_ids: Optional[
            List[int]
        ]  # None means this is from the initial population

    __database: Database

    __evolutionary_optimizer_id: int

    __genotype_type: Type[Genotype]
    __fitness_type: Type[Fitness]
    __offspring_size: int

    __process_id_gen: ProcessIdGen

    __next_individual_id: int

    __latest_population: List[__Individual[Genotype]]
    __latest_fitnesses: Optional[List[Fitness]]  # None only for the initial population
    __generation_index: int

    async def ainit_new(
        self,
        database: Database,
        process_id: int,
        process_id_gen: ProcessIdGen,
        genotype_type: Type[Genotype],
        fitness_type: Type[Fitness],
        offspring_size: int,
        initial_population: List[Genotype],
    ) -> None:
        """
        :id: Unique id between all EvolutionaryOptimizers in this database.
        :offspring_size: Number of offspring made by the population each generation.
        """
        self.__database = database
        self.__genotype_type = genotype_type
        self.__fitness_type = fitness_type
        self.__offspring_size = offspring_size
        self.__process_id_gen = process_id_gen
        self.__next_individual_id = 0
        self.__latest_fitnesses = None
        self.__generation_index = 0

        self.__latest_population = [
            self.__Individual(self.__gen_next_individual_id(), g, None)
            for g in initial_population
        ]

        async with self.__database.engine.begin() as conn:
            await conn.run_sync(DbBase.metadata.create_all)

        async with self.__database.session() as session:
            async with session.begin():
                new_opt = DbEvolutionaryOptimizer(
                    process_id=process_id,
                    offspring_size=self.__offspring_size,
                    genotype_type="GENOTYPE_TEST",  # TODO proper genotype&fitness types
                    fitness_type="FITNESS_TEST",
                )
                session.add(new_opt)
                await session.flush()
                self.__evolutionary_optimizer_id = new_opt.id

                # TODO prepare genotype table
                # TODO prepare fitness table

                session.add(
                    DbEvolutionaryOptimizerState(
                        evolutionary_optimizer_id=self.__evolutionary_optimizer_id,
                        generation_index=self.__generation_index,
                        processid_state=self.__process_id_gen.get_state(),
                    )
                )

                session.add_all(
                    [
                        DbEvolutionaryOptimizerIndividual(
                            evolutionary_optimizer_id=self.__evolutionary_optimizer_id,
                            individual_id=i.id,
                            genotype_id=0,  # TODO fitness and genotype
                            fitness_id=None,  # will set after evaluating first generation
                        )
                        for i in self.__latest_population
                    ]
                )

                session.add_all(
                    [
                        DbEvolutionaryOptimizerGeneration(
                            evolutionary_optimizer_id=self.__evolutionary_optimizer_id,
                            generation_index=self.__generation_index,
                            individual_index=index,
                            individual_id=individual.id,
                        )
                        for index, individual in enumerate(self.__latest_population)
                    ]
                )

    async def ainit_from_database(
        self,
        database: Database,
        process_id: int,
        process_id_gen: ProcessIdGen,
        genotype_type: Type[Genotype],
        fitness_type: Type[Fitness],
    ) -> bool:
        self.__database = database
        self.__genotype_type = genotype_type
        self.__fitness_type = fitness_type

        async with database.session() as session:
            try:
                (eo_row,) = (
                    await session.execute(
                        select(DbEvolutionaryOptimizer).filter(
                            DbEvolutionaryOptimizer.process_id == process_id
                        )
                    )
                ).one()
            except MultipleResultsFound as err:
                raise IncompatibleError() from err
            except (NoResultFound, OperationalError):
                return False

            self.__evolutionary_optimizer_id = eo_row.id
            self.__offspring_size = eo_row.offspring_size

            (gen_row,) = (
                await session.execute(
                    select(DbEvolutionaryOptimizerGeneration)
                    .filter(
                        DbEvolutionaryOptimizerGeneration.id
                        == self.__evolutionary_optimizer_id
                    )
                    .order_by(DbEvolutionaryOptimizerGeneration.generation_index.desc())
                )
            ).first()

            if gen_row is None:
                raise IncompatibleError()  # not possible that there is no generation but DbEvolutionaryOptimizer row exists

            self.__generation_index = gen_row.generation_index

            self.__process_id_gen = process_id_gen
            self.__process_id_gen.set_state(gen_row.processid_state_before_evaluation)

            # TODO load individuals
            # TODO load __latest_fitnesses
            # self.__latest_population = []  # TODO
            # self.__latest_fitnesses = []  # TODO

        return True

    async def run(self) -> None:
        # evaluate initial population if required
        if self.__latest_fitnesses is None:
            self.__latest_fitnesses = await self.__safe_evaluate_generation(
                [i.genotype for i in self.__latest_population],
                self.__database,
                self.__process_id_gen.gen(),
                self.__process_id_gen,
            )
            initial_fitnesses = self.__latest_fitnesses
        else:
            initial_fitnesses = None

        while self.__safe_must_do_next_gen():

            # let user select parents
            parent_selections = self.__safe_select_parents(
                [i.genotype for i in self.__latest_population],
                self.__latest_fitnesses,
                self.__offspring_size,
            )

            # let user create offspring
            offspring = [
                self.__safe_mutate(
                    self.__safe_crossover(
                        [self.__latest_population[i].genotype for i in s]
                    )
                )
                for s in parent_selections
            ]

            # let user evaluate offspring
            new_fitnesses = await self.__safe_evaluate_generation(
                offspring,
                self.__database,
                self.__process_id_gen.gen(),
                self.__process_id_gen,
            )

            # combine to create list of individuals
            new_individuals = [
                self.__Individual(
                    self.__gen_next_individual_id(),
                    genotype,
                    [self.__latest_population[i].id for i in parent_indices],
                )
                for parent_indices, genotype in zip(parent_selections, offspring)
            ]

            # let user select survivors between old and new individuals
            old_survivors, new_survivors = self.__safe_select_survivors(
                self.__latest_population,
                self.__latest_fitnesses,
                new_individuals,
                new_fitnesses,
                len(self.__latest_population),
            )

            survived_new_individuals = [new_individuals[i] for i in new_survivors]
            survived_new_fitnesses = [new_fitnesses[i] for i in new_survivors]

            # combine old and new and store as the new generation
            self.__latest_population = [
                self.__latest_population[i] for i in old_survivors
            ] + survived_new_individuals

            self.__latest_fitnesses = [
                self.__latest_fitnesses[i] for i in old_survivors
            ] + survived_new_fitnesses

            self.__generation_index += 1

            # save generation and possibly fitnesses of initial population
            await self.__save_generation(
                initial_fitnesses, survived_new_individuals, survived_new_fitnesses
            )

        assert (
            self.__generation_index > 0
        ), "Must create at least one generation beyond initial population. This behaviour is not supported."  # would break database structure

    @property
    def generation_index(self) -> Optional[int]:
        """
        Get the current generation.
        The initial generation is numbered 0.
        """

        return self.__generation_index

    def __gen_next_individual_id(self) -> int:
        next_id = self.__next_individual_id
        self.__next_individual_id += 1
        return next_id

    async def __safe_evaluate_generation(
        self,
        genotypes: List[Genotype],
        database: Database,
        process_id: int,
        process_id_gen: ProcessIdGen,
    ) -> List[Fitness]:
        fitnesses = await self._evaluate_generation(
            genotypes=genotypes,
            database=database,
            process_id=process_id,
            process_id_gen=process_id_gen,
        )
        assert type(fitnesses) == list
        assert len(fitnesses) == len(genotypes)
        assert all(type(e) == self.__fitness_type for e in fitnesses)
        return fitnesses

    def __safe_select_parents(
        self,
        population: List[Genotype],
        fitnesses: List[Fitness],
        num_parent_groups: int,
    ) -> List[List[int]]:
        parent_selections = self._select_parents(
            population, fitnesses, num_parent_groups
        )
        assert type(parent_selections) == list
        assert len(parent_selections) == num_parent_groups
        assert all(type(s) == list for s in parent_selections)
        assert all(
            [
                all(type(p) == int and p >= 0 and p < len(population) for p in s)
                for s in parent_selections
            ]
        )
        return parent_selections

    def __safe_crossover(self, parents: List[Genotype]) -> Genotype:
        genotype = self._crossover(parents)
        assert type(genotype) == self.__genotype_type
        return genotype

    def __safe_mutate(self, genotype: Genotype) -> Genotype:
        genotype = self._mutate(genotype)
        assert type(genotype) == self.__genotype_type
        return genotype

    def __safe_select_survivors(
        self,
        old_individuals: List[Genotype],
        old_fitnesses: List[Fitness],
        new_individuals: List[Genotype],
        new_fitnesses: List[Fitness],
        num_survivors: int,
    ) -> Tuple(List[int], List[int]):
        old_survivors, new_survivors = self._select_survivors(
            old_individuals,
            old_fitnesses,
            new_individuals,
            new_fitnesses,
            num_survivors,
        )
        assert type(old_survivors) == list
        assert type(new_survivors) == list
        assert len(old_survivors) + len(new_survivors) == len(self.__latest_population)
        assert all(type(s) == int for s in old_survivors)
        assert all(type(s) == int for s in new_survivors)
        return (old_survivors, new_survivors)

    def __safe_must_do_next_gen(self) -> bool:
        must_do = self._must_do_next_gen()
        assert type(must_do) == bool
        return must_do

    async def __save_generation(
        self,
        initial_fitnesses: Optional[List[Fitness]],
        new_individuals: List[__Individual],
        new_fitnesses: List[Fitness],
    ) -> None:
        # TODO set initial fitnesses
        async with self.__database.session() as session:
            async with session.begin():
                session.add(
                    DbEvolutionaryOptimizerState(
                        evolutionary_optimizer_id=self.__evolutionary_optimizer_id,
                        generation_index=self.__generation_index,
                        processid_state=self.__process_id_gen.get_state(),
                    )
                )

                session.add_all(
                    [
                        DbEvolutionaryOptimizerIndividual(
                            evolutionary_optimizer_id=self.__evolutionary_optimizer_id,
                            individual_id=i.id,
                            genotype_id=0,  # TODO fitness and genotype
                            fitness_id=0,
                        )
                        for i in new_individuals
                    ]
                )

                session.add_all(
                    [
                        DbEvolutionaryOptimizerGeneration(
                            evolutionary_optimizer_id=self.__evolutionary_optimizer_id,
                            generation_index=self.__generation_index,
                            individual_index=index,
                            individual_id=individual.id,
                        )
                        for index, individual in enumerate(self.__latest_population)
                    ]
                )
