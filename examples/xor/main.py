from __future__ import annotations

import pickle
import random
from typing import List, Optional

import numpy as np
import sqlalchemy
from revolve2.core.database import open_async_database_sqlite
from revolve2.core.optimization import DbId
from revolve2.core.optimization.ea.population import (
    make_measures,
    make_serializable,
    serializable_list_template,
)
from revolve2.core.optimization.ea.population.pop_list import (
    multiple_unique,
    pop_list_template,
    topn,
    tournament,
)
from sqlalchemy import Column, Integer, String
from sqlalchemy.ext.asyncio import AsyncEngine
from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.future import select
from sqlalchemy.orm import declarative_base

ParamList = serializable_list_template(float, "parameters", "parameter")


@make_serializable(table_name="genotype")
class Genotype:
    params: ParamList


@make_measures(table_name="measures")
class Measures:
    displacement: Optional[float] = None


Population, Individual = pop_list_template("population", Genotype, Measures)


class Optimizer:
    POPULATION_SIZE: int = 100
    OFFSPRING_SIZE: int = 50

    dbid: DbId
    db: AsyncEngine
    rng: np.random.Generator
    pop: Population
    gen_index: int

    optimizer_id: Column[Integer]

    async def run(self) -> None:
        self.dbid = DbId.root("opt")

        self.db = open_async_database_sqlite("database")
        async with self.db.begin() as conn:
            await conn.run_sync(DbBase.metadata.create_all)
            await Population.prepare_db(conn)

        if not await self.load_state():
            self.rng = np.random.Generator(np.random.PCG64(0))
            self.pop = Population(
                [
                    Individual(Genotype(params=ParamList([1.0, 2.0, 3.0])), Measures())
                    for _ in range(self.POPULATION_SIZE)
                ]
            )
            self.gen_index = 0
            self.measure(self.pop)

            async with AsyncSession(self.db) as ses:
                async with ses.begin():
                    dbopt = DbOptimizer(db_id=self.dbid.fullname)
                    ses.add(dbopt)
                    await ses.flush()
                    self.optimizer_id = dbopt.id
                    await self.save_state(ses)

        while self.gen_index < 100:
            self.evolve()
            async with AsyncSession(self.db) as ses:
                async with ses.begin():
                    await self.save_state(ses)

    async def load_state(self) -> bool:
        async with AsyncSession(self.db) as ses:
            async with ses.begin():
                dbopt = (
                    await ses.execute(
                        select(DbOptimizer).filter(
                            DbOptimizer.db_id == self.dbid.fullname
                        )
                    )
                ).scalar_one_or_none()

                if dbopt is None:
                    return False

                self.optimizer_id = dbopt.id

                state = (
                    await ses.execute(
                        select(DbState)
                        .filter(DbState.optimizer_id == self.optimizer_id)
                        .order_by(DbState.generation_index.desc())
                        .limit(1)
                    )
                ).scalar_one()

                self.gen_index = state.generation_index
                self.rng = pickle.loads(state.rng_pickled)

                self.pop = await Population.from_db(ses, state.pop_id)

                return True

    async def save_state(self, ses: AsyncSession) -> None:
        popid = await self.pop.to_db(ses)

        dbstate = DbState(
            optimizer_id=self.optimizer_id,
            generation_index=self.gen_index,
            rng_pickled=pickle.dumps(self.rng),
            pop_id=popid,
        )
        ses.add(dbstate)

    def evolve(self) -> None:
        self.gen_index += 1

        parent_groups = [
            multiple_unique(
                self.pop, 2, lambda pop: tournament(pop, "displacement", self.rng, k=2)
            )
            for _ in range(self.OFFSPRING_SIZE)
        ]

        offspring = Population(
            [
                Individual(
                    self.mutate(
                        self.crossover(
                            self.pop[parents[0]],
                            self.pop[parents[1]],
                        )
                    ),
                    Measures(),
                )
                for parents in parent_groups
            ]
        )
        self.measure(offspring)

        original_selection, offspring_selection = topn(
            self.pop, offspring, measure="displacement", n=self.POPULATION_SIZE
        )

        self.pop = Population.from_existing_populations(
            [self.pop, offspring],
            [original_selection, offspring_selection],
            ["displacement"],
        )

    def mutate(self, genotype: Genotype) -> Genotype:
        return Genotype(params=ParamList([1.0, 2.0, 3.0]))  # TODO

    def crossover(self, parent1: Genotype, parent2: Genotype) -> Genotype:
        return Genotype(params=ParamList([1.0, 2.0, 3.0]))  # TODO

    def measure_displacements(self, genotypes: List[Genotype]) -> List[float]:
        return [random.randint(0, 1000) for _ in genotypes]  # TODO

    def measure(self, pop: Population) -> None:
        displacements = self.measure_displacements([i.genotype for i in pop])
        for individual, displacement in zip(pop, displacements):
            individual.measures["displacement"] = displacement


DbBase = declarative_base()


class DbOptimizer(DbBase):
    __tablename__ = "optimizer"

    id = Column(
        Integer,
        nullable=False,
        unique=True,
        autoincrement=True,
        primary_key=True,
    )
    db_id = Column(String, nullable=False, unique=True)


class DbState(DbBase):
    __tablename__ = "state"

    id = Column(
        Integer,
        nullable=False,
        unique=True,
        autoincrement=True,
        primary_key=True,
    )
    optimizer_id = Column(
        Integer, sqlalchemy.ForeignKey(DbOptimizer.id), nullable=False
    )
    generation_index = Column(Integer, nullable=False)
    rng_pickled = Column(String, nullable=False)
    pop_id = Column(Integer, sqlalchemy.ForeignKey(Population.table.id), nullable=False)


async def main() -> None:
    await Optimizer().run()


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
