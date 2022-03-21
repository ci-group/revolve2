from typing import TypeVar, Generic, Type, Optional
from abc import abstractmethod, ABC
from revolve2.core.database import Database
from sqlalchemy.ext.asyncio.session import AsyncSession

ChildClass = TypeVar("ChildClass", bound="Process")


class Process(ABC, Generic[ChildClass]):
    def __init__(self) -> None:
        raise RuntimeError("Call 'new' or 'from_database' instead.")

    @abstractmethod
    async def ainit_new(
        self,
        database: Database,
        session: AsyncSession,
        process_id: int,
        *args,
        **kwargs
    ) -> None:
        """
        Init called when creating an instance through 'new'.
        Use the session to save initial settings that could be loaded using `ainit_from_database`.
        The session must not be commit, but it may be flushed.
        """
        pass

    @abstractmethod
    async def ainit_from_database(
        self, database: Database, process_id: int, *args, **kwargs
    ) -> bool:
        """
        Init called when creating an instance through 'from_database'.
        """
        pass

    @classmethod
    async def new(
        cls: Type[ChildClass], database: Database, process_id: int, *args, **kwargs
    ) -> ChildClass:
        self = super().__new__(cls)
        async with database.session() as session:
            async with session.begin():
                await self.ainit_new(database, session, process_id, *args, **kwargs)
        return self

    @classmethod
    async def from_database(
        cls: Type[ChildClass], database: Database, process_id: int, *args, **kwargs
    ) -> Optional[ChildClass]:
        """
        Create instance and initialize from database.
        :returns: Instance if init from database was possible or None.
                  None means there was no data in the database to initialize this object.
        :raises SerializeError: If database incompatible.
        """
        self = super().__new__(cls)
        if await self.ainit_from_database(database, process_id, *args, **kwargs):
            return self
        else:
            return None
