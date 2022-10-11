from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, List, Optional, Type, TypeVar

from sqlalchemy.ext.asyncio import AsyncConnection
from sqlalchemy.ext.asyncio.session import AsyncSession

SerializableSelf = TypeVar("SerializableSelf", bound="Serializable")


class Serializable(ABC):
    """Interface for classes that can be serialized to a database."""

    table: Any  # TODO

    @classmethod
    @abstractmethod
    async def prepare_db(cls, conn: AsyncConnection) -> None:
        """
        Set up the database, creating tables.

        :param conn: Connection to the database.
        """
        pass

    async def to_db(self: SerializableSelf, ses: AsyncSession) -> int:
        """
        Serialize this object to a database.

        :param ses: Database session.
        :returns: Id of the object in the database.
        """
        return (await self.to_db_multiple(ses, [self]))[0]

    @classmethod
    @abstractmethod
    async def to_db_multiple(
        cls: Type[SerializableSelf], ses: AsyncSession, objects: List[SerializableSelf]
    ) -> List[int]:
        """
        Serialize multiple objects to a database.

        :param ses: Database session.
        :param objects: The objects to serialize.
        :returns: Ids of the objects in the database.
        """
        pass

    @classmethod
    @abstractmethod
    async def from_db(
        cls: Type[SerializableSelf], ses: AsyncSession, id: int
    ) -> Optional[SerializableSelf]:
        """
        Deserialize this object from a database.

        If id does not exist, returns None.

        :param ses: Database session.
        :param id: Id of the object in the database.
        :returns: The deserialized object or None is id does not exist.
        """
        pass
