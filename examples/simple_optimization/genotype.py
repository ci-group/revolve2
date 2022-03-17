from __future__ import annotations

from revolve2.core.optimization.ea import Genotype as GenotypeInterface
import sqlalchemy
from typing import List
from random import Random
from phenotype import Phenotype
from revolve2.core.database import Database
from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.ext.declarative import declarative_base
from item import Item


class Genotype(GenotypeInterface["Genotype"]):
    items: List[bool]

    def __init__(self, items: List[bool]) -> None:
        self.items = items

    @classmethod
    def random(cls, rng: Random, has_item_prob: float, num_items: int) -> Genotype:
        return cls([rng.random() < has_item_prob for _ in range(num_items)])

    def develop(self, items: List[Item], maximum_weight: float) -> Phenotype:
        phenotype = []
        total_weight = 0
        for has_item, item in zip(self.items, items):
            if has_item and total_weight + item.weight < maximum_weight:
                phenotype.append(True)
            else:
                phenotype.append(False)

        return Phenotype(phenotype)

    @classmethod
    async def create_tables(cls, database: Database) -> None:
        async with database.engine.begin() as conn:
            await conn.run_sync(DbGenotype.metadata.create_all)

    @classmethod
    def identifying_table(cls) -> str:
        return DbGenotype.__tablename__

    @classmethod
    async def to_database(
        cls, session: AsyncSession, objects: List[Genotype]
    ) -> List[int]:
        dbfitnesses = [
            DbGenotype(items="".join(["1" if x else "0" for x in g.items]))
            for g in objects
        ]
        session.add_all(dbfitnesses)
        await session.flush()
        return [dbfitness.id for dbfitness in dbfitnesses]

    @classmethod
    async def from_database(cls, session: AsyncSession, ids: List[int]) -> Genotype:
        raise NotImplementedError()


DbBase = declarative_base()


class DbGenotype(DbBase):
    __tablename__ = "genotype"

    id = sqlalchemy.Column(
        sqlalchemy.Integer,
        nullable=False,
        unique=True,
        autoincrement=True,
        primary_key=True,
    )
    items = sqlalchemy.Column(sqlalchemy.String, nullable=False)
