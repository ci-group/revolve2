from abc import ABC, abstractmethod
from typing import Generic, List, TypeVar

from sqlalchemy.ext.asyncio.session import AsyncSession

T = TypeVar("T")


class Serializer(ABC, Generic[T]):
    """An interface for classes that can serialize class T."""

    @classmethod
    @abstractmethod
    async def create_tables(cls, session: AsyncSession) -> None:
        """
        Create all tables required for storing objects of type T.

        :param session: Database session used for creating the tables. TODO make sure to require this will not commit. It currently often will.
        """

    @classmethod
    @abstractmethod
    def identifying_table(cls) -> str:
        """
        Get the name of the primary table used for storing objects of type T.

        :returns: The name of the primary table.
        """

    @classmethod
    @abstractmethod
    async def to_database(cls, session: AsyncSession, objects: List[T]) -> List[int]:
        """
        Serialize the provided objects to a database using the provided session.

        :param session: Session used when serializing to the database. Implementor must make sure this is not committed.
        :param objects: The objects to serialize.
        :returns: A list of ids to identify each serialized object.
        """

    @classmethod
    @abstractmethod
    async def from_database(cls, session: AsyncSession, ids: List[int]) -> List[T]:
        """
        Deserialize a list of objects from a database using the provided session.

        :param session: Session used for deserialization from the database. Implementor must make sure no changes are made to the database.
        :param ids: Ids identifying the objects to deserialize.
        :returns: The deserialized objects.
        :raises IncompatibleError: In case the database is not compatible with this serializer.
        """
