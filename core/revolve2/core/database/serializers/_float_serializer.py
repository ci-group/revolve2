from __future__ import annotations

from typing import List

import sqlalchemy
from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.future import select

from .._serializer import Serializer


class FloatSerializer(Serializer[float]):
    """Serializer for storing generic floats."""

    @classmethod
    async def create_tables(cls, session: AsyncSession) -> None:
        """
        Create all tables required for serialization.

        This function commits. TODO fix this
        :param session: Database session used for creating the tables.
        """
        await (await session.connection()).run_sync(DbBase.metadata.create_all)

    @classmethod
    def identifying_table(cls) -> str:
        """
        Get the name of the primary table used for storing the floats.

        :returns: The name of the primary table.
        """
        return DbFloat.__tablename__

    @classmethod
    async def to_database(
        cls, session: AsyncSession, objects: List[float]
    ) -> List[int]:
        """
        Serialize the provided floats to a database using the provided session.

        :param session: Session used when serializing to the database. This session will not be committed by this function.
        :param objects: The floats to serialize.
        :return: A list of ids to identify each serialized float.
        """
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
        """
        Deserialize a list of floats from a database using the provided session.

        :param session: Session used for deserialization from the database. No changes are made to the database.
        :param ids: Ids identifying the floats to deserialize.
        :returns: The deserialized floats.
        """
        items = (
            (await session.execute(select(DbFloat).filter(DbFloat.id.in_(ids))))
            .scalars()
            .all()
        )

        idmap = {item.id: item for item in items}

        return [idmap[id].value for id in ids]


DbBase = declarative_base()


class DbFloat(DbBase):
    """Stores all floats."""

    __tablename__ = "float"

    id = sqlalchemy.Column(
        sqlalchemy.Integer, nullable=False, primary_key=True, autoincrement=True
    )
    value = sqlalchemy.Column(sqlalchemy.Float, nullable=False)
