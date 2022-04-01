from abc import ABC, abstractmethod
from typing import Generic, List, TypeVar, Union

from sqlalchemy.ext.asyncio.session import AsyncSession

T = TypeVar("T")


class Serializer(ABC, Generic[T]):
    @classmethod
    @abstractmethod
    async def create_tables(cls, session: AsyncSession) -> None:
        """
        Creates all tables required for storing objects of type T.
        """

    @classmethod
    @abstractmethod
    def identifying_table(cls) -> str:
        """
        Returns the name of the primary table used for storing objects of type T.
        """

    @classmethod
    @abstractmethod
    async def to_database(cls, session: AsyncSession, objects: List[T]) -> List[int]:
        """
        Serializes the object to a database using the provided session.
        """

    @classmethod
    @abstractmethod
    async def from_database(cls, session: AsyncSession, ids: List[int]) -> List[T]:
        """
        Deserializes an object from a database using the provided session.
        """
