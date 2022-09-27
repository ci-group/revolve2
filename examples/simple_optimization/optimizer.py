"""Optimizer for knapsack problem."""

from __future__ import annotations

import pickle
from random import Random
from typing import List, Tuple

import revolve2.core.optimization.ea.generic_ea.population_management as population_management
import revolve2.core.optimization.ea.generic_ea.selection as selection
import sqlalchemy
from genotype import Genotype, GenotypeSerializer, develop
from item import Item
from revolve2.core.database import IncompatibleError
from revolve2.core.database.serializers import FloatSerializer
from revolve2.core.optimization import DbId
from revolve2.core.optimization.ea.generic_ea import EAOptimizer
from sqlalchemy.ext.asyncio import AsyncEngine
from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.future import select


class Optimizer(EAOptimizer[Genotype, float]):
    """
    Optimizer for knapsack problem.

    Uses the generic EA optimizer as a base.
    """

    _db_id: DbId
    _rng: Random
    _items: List[Item]
    _max_weight: float
    _num_generations: int

    async def ainit_new(  # type: ignore # TODO for now ignoring mypy complaint about LSP problem, override parent's ainit
        self,
        database: AsyncEngine,
        session: AsyncSession,
        db_id: DbId,
        offspring_size: int,
        initial_population: List[Genotype],
        rng: Random,
        items: List[Item],
        max_weight: float,
        num_generations: int,
    ) -> None:
        """
        Initialize this class async.

        Called when creating an instance using `new`.

        :param database: Database to use for this optimizer.
        :param session: Session to use when saving data to the database during initialization.
        :param db_id: Unique identifier in the completely program specifically made for this optimizer.
        :param offspring_size: Number of offspring made by the population each generation.
        :param initial_population: List of genotypes forming generation 0.
        :param rng: Random number generator.
        :param items: The items that could be in the knapsack.
        :param max_weight: Maximum weight of the knapsack.
        :param num_generations: Number of generation to run the optimizer for.
        """
        await super().ainit_new(
            database=database,
            session=session,
            db_id=db_id,
            genotype_type=Genotype,
            genotype_serializer=GenotypeSerializer,
            fitness_type=float,
            fitness_serializer=FloatSerializer,
            offspring_size=offspring_size,
            initial_population=initial_population,
        )

        self._db_id = db_id
        self._rng = rng
        self._items = items
        self._max_weight = max_weight
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
        db_id: DbId,
        rng: Random,
        items: List[Item],
        max_weight: float,
        num_generations: int,
    ) -> bool:
        """
        Try to initialize this class async from a database.

        Called when creating an instance using `from_database`.

        :param database: Database to use for this optimizer.
        :param session: Session to use when loading and saving data to the database during initialization.
        :param db_id: Unique identifier in the completely program specifically made for this optimizer.
        :param rng: Random number generator.
        :param items: The items that could be in the knapsack.
        :param max_weight: Maximum weight of the knapsack.
        :param num_generations: Number of generation to run the optimizer for.
        :returns: True if this complete object could be deserialized from the database.
        :raises IncompatibleError: In case the database is not compatible with this class.
        """
        if not await super().ainit_from_database(
            database=database,
            session=session,
            db_id=db_id,
            genotype_type=Genotype,
            genotype_serializer=GenotypeSerializer,
            fitness_type=float,
            fitness_serializer=FloatSerializer,
        ):
            return False

        self._db_id = db_id
        self._items = items
        self._max_weight = max_weight
        self._num_generations = num_generations

        opt_row = (
            (
                await session.execute(
                    select(DbOptimizerState)
                    .filter(DbOptimizerState.db_id == db_id.fullname)
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
        db_id: DbId,
    ) -> List[float]:
        phenotypes = [
            develop(genotype, self._items, self._max_weight) for genotype in genotypes
        ]
        return [
            float(
                sum(
                    [
                        has_items * item.value
                        for has_items, item in zip(phenotype.items, self._items)
                    ]
                )
            )
            for phenotype in phenotypes
        ]

    def _select_parents(
        self,
        population: List[Genotype],
        fitnesses: List[float],
        num_parent_groups: int,
    ) -> List[List[int]]:
        return [
            selection.multiple_unique(
                2,
                population,
                fitnesses,
                lambda _, fitnesses: selection.tournament(self._rng, fitnesses, k=2),
            )
            for _ in range(num_parent_groups)
        ]

    def _select_survivors(
        self,
        old_individuals: List[Genotype],
        old_fitnesses: List[float],
        new_individuals: List[Genotype],
        new_fitnesses: List[float],
        num_survivors: int,
    ) -> Tuple[List[int], List[int]]:
        assert len(old_individuals) == num_survivors

        return population_management.steady_state(
            old_individuals,
            old_fitnesses,
            new_individuals,
            new_fitnesses,
            selection.topn,
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
                db_id=self._db_id.fullname,
                generation_index=self.generation_index,
                rng=pickle.dumps(self._rng.getstate()),
            )
        )


DbBase = declarative_base()


class DbOptimizerState(DbBase):
    """State of the optimizer."""

    __tablename__ = "optimizer_state"

    db_id = sqlalchemy.Column(
        sqlalchemy.String,
        nullable=False,
        primary_key=True,
    )
    generation_index = sqlalchemy.Column(
        sqlalchemy.Integer, nullable=False, primary_key=True
    )
    rng = sqlalchemy.Column(sqlalchemy.PickleType, nullable=False)
