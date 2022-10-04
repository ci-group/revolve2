from typing import Protocol, Type, TypeVar

from sqlalchemy.ext.asyncio import AsyncConnection
from sqlalchemy.ext.asyncio.session import AsyncSession

Self = TypeVar("Self", bound="DbSerializable")


class DbSerializable(Protocol):
    @classmethod
    async def prepare_db(cls, conn: AsyncConnection) -> None:
        pass

    async def to_db(
        self: Self,
        ses: AsyncSession,
    ) -> int:
        pass

    @classmethod
    async def from_db(cls: Type[Self], ses: AsyncSession, id: int) -> Self:
        pass
