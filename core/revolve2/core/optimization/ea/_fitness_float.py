from __future__ import annotations

from typing import List

from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.future import select

from revolve2.core.database import IncompatibleError, Tableable

from .fitness_float_schema import DbFitnessFloat


class FitnessFloat(float, Tableable):
    @classmethod
    async def create_tables(cls, session: AsyncSession) -> None:
        await (await session.connection()).run_sync(DbFitnessFloat.metadata.create_all)

    @classmethod
    def identifying_table(cls) -> str:
        return DbFitnessFloat.__tablename__

    @classmethod
    async def to_database(
        cls, session: AsyncSession, objects: List[FitnessFloat]
    ) -> List[int]:
        dbfitnesses = [DbFitnessFloat(fitness=fitness) for fitness in objects]
        session.add_all(dbfitnesses)
        await session.flush()
        ids = [
            dbfitness.id for dbfitness in dbfitnesses if dbfitness.id is not None
        ]  # cannot be none because not nullable but adding check for mypy
        assert len(ids) == len(objects)  # just to be sure because of check above
        return ids

    @classmethod
    async def from_database(
        cls, session: AsyncSession, ids: List[int]
    ) -> List[FitnessFloat]:
        rows = (
            (
                await session.execute(
                    select(DbFitnessFloat).filter(DbFitnessFloat.id.in_(ids))
                )
            )
            .scalars()
            .all()
        )

        if len(rows) != len(ids):
            raise IncompatibleError()

        id_map = {t.id: t for t in rows}
        return [FitnessFloat(id_map[id].fitness) for id in ids]
