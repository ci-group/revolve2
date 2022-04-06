from __future__ import annotations

from typing import List, cast

import sqlalchemy
from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.future import select

from .._serializer import Serializer


class FloatSerializer(Serializer[float]):
    @classmethod
    async def create_tables(cls, session: AsyncSession) -> None:
        await (await session.connection()).run_sync(DbBase.metadata.create_all)

    @classmethod
    def identifying_table(cls) -> str:
        return DbFloat.__tablename__

    @classmethod
    async def to_database(
        cls, session: AsyncSession, objects: List[float]
    ) -> List[int]:
        items = [DbFloat(value=f) for f in objects]
        session.add_all(items)
        await session.flush()

        res = [
            i.id for i in items if i.id is not None
        ]  # is not None only there to silence mypy. can not actually be none because is marked not nullable.
        assert len(res) == len(objects)  # just to be sure now that we do the above

        return res

    @classmethod
    async def from_database(cls, session: AsyncSession, ids: List[int]) -> List[float]:
        items = (
            (await session.execute(select(DbFloat).filter(DbFloat.id.in_(ids))))
            .scalars()
            .all()
        )

        idmap = {item.id: item for item in items}

        return [idmap[id].value for id in ids]


DbBase = declarative_base()


class DbFloat(DbBase):
    __tablename__ = "float"

    id = sqlalchemy.Column(
        sqlalchemy.Integer, nullable=False, primary_key=True, autoincrement=True
    )
    value = sqlalchemy.Column(sqlalchemy.Float, nullable=False)
