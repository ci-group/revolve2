from __future__ import annotations

from revolve2.core.optimization.ea import Fitness as FitnessInterface
import sqlalchemy
from sqlalchemy.ext.asyncio.session import AsyncSession
from revolve2.core.database import Database
from typing import List
from sqlalchemy.ext.declarative import declarative_base
from revolve2.core.database import IncompatibleError
from sqlalchemy.future import select
from sqlalchemy.orm.exc import NoResultFound
from sqlalchemy.orm.exc import MultipleResultsFound


class Fitness(float, FitnessInterface["Fitness"]):
    @classmethod
    async def create_tables(cls, database: Database) -> None:
        async with database.engine.begin() as conn:
            await conn.run_sync(DbFitness.metadata.create_all)

    @classmethod
    def identifying_table(cls) -> str:
        return DbFitness.__tablename__

    @classmethod
    async def to_database(
        cls, session: AsyncSession, objects: List[Fitness]
    ) -> List[int]:
        dbfitnesses = [DbFitness(fitness=fitness) for fitness in objects]
        session.add_all(dbfitnesses)
        await session.flush()
        return [dbfitness.id for dbfitness in dbfitnesses]

    @classmethod
    async def from_database(cls, session: AsyncSession, ids: List[int]) -> Fitness:
        try:
            rows = (
                (await session.execute(select(DbFitness).filter(DbFitness.id.in_(ids))))
                .scalars()
                .all()
            )
        except MultipleResultsFound as err:
            raise IncompatibleError() from err
        except NoResultFound:
            return False

        if len(rows) != len(ids):
            raise IncompatibleError()

        id_map = {t.id: t for t in rows}
        return [Fitness(id_map[id].fitness) for id in ids]


DbBase = declarative_base()


class DbFitness(DbBase):
    __tablename__ = "fitness"

    id = sqlalchemy.Column(
        sqlalchemy.Integer,
        nullable=False,
        unique=True,
        autoincrement=True,
        primary_key=True,
    )
    fitness = sqlalchemy.Column(sqlalchemy.Integer, nullable=False)
