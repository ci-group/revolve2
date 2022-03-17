from revolve2.core.database import Database
from abc import ABC, abstractmethod
from typing import TypeVar, Generic, List
from sqlalchemy.ext.asyncio.session import AsyncSession

Child = TypeVar("Child")


class Genotype(ABC, Generic[Child]):
    @classmethod
    @abstractmethod
    async def create_tables(cls, database: Database) -> None:
        """
        Create all tables required for storing this class.
        """

    @classmethod
    @abstractmethod
    def identifying_table(cls) -> str:
        """
        Return the name of the table that can be used to find stored object manually using their id.
        """

    @classmethod
    @abstractmethod
    async def to_database(
        cls, session: AsyncSession, objects: List[Child]
    ) -> List[int]:
        """
        Save a list of these objects to the database.
        Return identifiers that can be used to load them using `from_database`.
        Multiple calls to this function must generate new entries and return new ids.
        You can flush but not commit the session.

        :database: Database to save to.
        :return: Identifiers that can be used to load objects from the database.
        """

    @classmethod
    @abstractmethod
    async def from_database(cls, session: AsyncSession, ids: List[int]) -> Child:
        """
        Load instances of this class from the database using ids generated by `to_database`.

        :database: Database to load from.
        :return: Identifiers created by `to_database`.
        """
