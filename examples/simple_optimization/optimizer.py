from __future__ import annotations

import pickle
from random import Random
from typing import List, Tuple

from genotype import Genotype
from item import Item
from optimizer_schema import DbBase, DbOptimizerState
from sqlalchemy.ext.asyncio import AsyncEngine
from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.future import select

import revolve2.core.optimization.ea.population_management as population_management
import revolve2.core.optimization.ea.selection as selection
from revolve2.core.database import IncompatibleError
from revolve2.core.optimization import ProcessIdGen
from revolve2.core.optimization.ea import EvolutionaryOptimizer, FitnessFloat


class Optimizer(EvolutionaryOptimizer[Genotype, FitnessFloat]):
    _process_id: int
    _rng: Random
    _items: List[Item]
    _num_generations: int

    async def ainit_new(  # type: ignore # TODO for now ignoring mypy complaint about LSP problem, override parent's ainit
        self,
        database: AsyncEngine,
        session: AsyncSession,
        process_id: int,
        process_id_gen: ProcessIdGen,
        offspring_size: int,
        initial_population: List[Genotype],
        rng: Random,
        items: List[Item],
        num_generations: int,
    ) -> None:
        await super().ainit_new(
            database=database,
            session=session,
            process_id=process_id,
            process_id_gen=process_id_gen,
            genotype_type=Genotype,
            fitness_type=FitnessFloat,
            offspring_size=offspring_size,
            initial_population=initial_population,
        )

        self._process_id = process_id
        self._rng = rng
        self._items = items
        self._num_generations = num_generations

        # create database structure if not exists
        # TODO this works but there is probably a better way
        await (await session.connection()).run_sync(DbBase.metadata.create_all)

        # save to database
        self._on_generation_checkpoint(session)

    async def ainit_from_database(  # type: ignore # see comment at ainit_new
        self,
        database: AsyncEngine,
        session: AsyncSession,
        process_id: int,
        process_id_gen: ProcessIdGen,
        rng: Random,
        items: List[Item],
        num_generations: int,
    ) -> bool:
        if not await super().ainit_from_database(
            database=database,
            session=session,
            process_id=process_id,
            process_id_gen=process_id_gen,
            genotype_type=Genotype,
            fitness_type=FitnessFloat,
        ):
            return False

        self._process_id = process_id
        self._items = items
        self._num_generations = num_generations

        opt_row = (
            (
                await session.execute(
                    select(DbOptimizerState)
                    .filter(DbOptimizerState.process_id == process_id)
                    .order_by(DbOptimizerState.generation_index.desc())
                )
            )
            .scalars()
            .first()
        )

        # if this happens something is wrong with the database
        if opt_row is None:
            raise IncompatibleError

        self._rng = rng
        self._rng.setstate(pickle.loads(opt_row.rng))

        return True

    async def _evaluate_generation(
        self,
        genotypes: List[Genotype],
        database: AsyncEngine,
        process_id: int,
        process_id_gen: ProcessIdGen,
    ) -> List[FitnessFloat]:
        return [
            FitnessFloat(
                sum(
                    [
                        has_items * item.value
                        for has_items, item in zip(genotype.items, self._items)
                    ]
                )
            )
            for genotype in genotypes
        ]

    def _select_parents(
        self,
        population: List[Genotype],
        fitnesses: List[FitnessFloat],
        num_parent_groups: int,
    ) -> List[List[int]]:
        return [
            selection.multiple_unique(
                population,
                fitnesses,
                2,
                lambda _, fitnesses: selection.tournament(self._rng, fitnesses, k=2),
            )
            for _ in range(num_parent_groups)
        ]

    def _select_survivors(
        self,
        old_individuals: List[Genotype],
        old_fitnesses: List[FitnessFloat],
        new_individuals: List[Genotype],
        new_fitnesses: List[FitnessFloat],
        num_survivors: int,
    ) -> Tuple[List[int], List[int]]:
        assert len(old_individuals) == num_survivors

        return population_management.steady_state(
            old_individuals,
            old_fitnesses,
            new_individuals,
            new_fitnesses,
            lambda _, fitnesses: selection.tournament(self._rng, fitnesses, k=2),
        )

    def _crossover(self, parents: List[Genotype]) -> Genotype:
        assert len(parents) == 2
        point = self._rng.randrange(0, len(parents[0].items))
        return Genotype(parents[0].items[0:point] + parents[1].items[point:])

    def _mutate(self, genotype: Genotype) -> Genotype:
        return Genotype(
            [
                has_item ^ (self._rng.random() < 1 / len(genotype.items))
                for has_item in genotype.items
            ]
        )

    def _must_do_next_gen(self) -> bool:
        return self.generation_index != self._num_generations

    def _on_generation_checkpoint(self, session: AsyncSession) -> None:
        session.add(
            DbOptimizerState(
                process_id=self._process_id,
                generation_index=self.generation_index,
                rng=pickle.dumps(self._rng.getstate()),
            )
        )
