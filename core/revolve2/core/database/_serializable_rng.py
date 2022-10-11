from __future__ import annotations

import pickle
from typing import Optional, Type

import numpy as np
from sqlalchemy import Column, Integer, LargeBinary
from sqlalchemy.ext.asyncio import AsyncConnection
from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.future import select
from sqlalchemy.orm import declarative_base

from ._serializable import Serializable

_DbBase = declarative_base()


class RngTable(_DbBase):
    """Main table for SerializableRng."""

    __tablename__ = "rng"

    id = Column(
        Integer,
        nullable=False,
        unique=True,
        autoincrement=True,
        primary_key=True,
    )
    pickled = Column(
        LargeBinary,
        nullable=False,
    )


class SerializableRng(Serializable):
    """Numpy Generator made Serializable."""

    table = RngTable

    rng: np.random.Generator

    def __init__(self, rng: np.random.Generator) -> None:
        """
        Initialize this object.

        :param rng: The numpy Generator instance to wrap.
        """
        self.rng = rng

    @classmethod
    async def prepare_db(cls, conn: AsyncConnection) -> None:
        """
        Set up the database, creating tables.

        :param conn: Connection to the database.
        """
        await conn.run_sync(_DbBase.metadata.create_all)

    async def to_db(
        self: SerializableRng,
        ses: AsyncSession,
    ) -> int:
        """
        Serialize this object to a database.

        :param ses: Database session.
        :returns: Id of the object in the database.
        """
        row = RngTable(pickled=pickle.dumps(self.rng))
        ses.add(row)
        await ses.flush()
        assert row.id is not None
        return row.id

    @classmethod
    async def from_db(
        cls: Type[SerializableRng], ses: AsyncSession, id: int
    ) -> Optional[SerializableRng]:
        """
        Deserialize this object from a database.

        If id does not exist, returns None.

        :param ses: Database session.
        :param id: Id of the object in the database.
        :returns: The deserialized object or None is id does not exist.
        """
        row = (
            await ses.execute(select(RngTable).filter(RngTable.id == id))
        ).scalar_one_or_none()

        if row is None:
            return None

        loaded = pickle.loads(row.pickled)
        assert isinstance(loaded, np.random.Generator)
        return SerializableRng(loaded)
