from typing import Protocol, Type, TypeVar

from sqlalchemy.ext.asyncio import AsyncConnection
from sqlalchemy.ext.asyncio.session import AsyncSession

Self = TypeVar("Self", bound="DbSerializable")


class DbSerializable(Protocol):
    """Protocol for classes that can be serialized to a database."""

    @classmethod
    async def prepare_db(cls, conn: AsyncConnection) -> None:
        """
        Set up the database, creating tables.

        :param conn: Connection to the database.
        """
        pass

    async def to_db(
        self: Self,
        ses: AsyncSession,
    ) -> int:
        """
        Serialize this object to a database.

        :param ses: Database session.
        :returns: Id of the object in the database.
        """
        pass

    @classmethod
    async def from_db(cls: Type[Self], ses: AsyncSession, id: int) -> Self:
        """
        Deserialize this object from a database.

        :param ses: Database session.
        :param id: Id of the object in the database.
        """
        pass
