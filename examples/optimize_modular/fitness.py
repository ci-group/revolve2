from __future__ import annotations
from revolve2.core.optimization.ea import Fitness as FitnessInterface
from revolve2.core.database import Database
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy import Integer, Column
from typing import List


class Fitness(float, FitnessInterface["Fitness"]):
    @classmethod
    async def create_tables(cls, database: Database) -> None:
        async with database.engine.begin() as conn:
            await conn.run_sync(DbBase.metadata.create_all)

    @classmethod
    async def to_database(cls, session, objects: List[Fitness]) -> List[int]:
        return [99 for _ in range(len(objects))]  # TODO

    @classmethod
    async def from_database(cls, session, ids: List[int]) -> Fitness:
        raise NotImplementedError()


DbBase = declarative_base()


class DbGenotype(DbBase):
    __tablename__ = "genotype"

    id = Column(
        Integer,
        nullable=False,
        unique=True,
        autoincrement=True,
        primary_key=True,
    )
    # TODO
