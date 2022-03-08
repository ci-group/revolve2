from typing import TypeVar, Generic, Type, Optional
from abc import ABC, abstractmethod
from revolve2.core.database import Database, Node

ChildClass = TypeVar("ChildClass", bound="Optimizer")


class Optimizer(ABC, Generic[ChildClass]):
    def __init__(self) -> None:
        raise RuntimeError("Call 'new' or 'from_database' instead.")

    @classmethod
    async def new(cls: Type[ChildClass], *args, **kwargs) -> ChildClass:
        self = super().__new__(cls)
        await self.ainit(*args, **kwargs)
        return self

    @classmethod
    async def from_database(
        cls: Type[ChildClass], database: Database, db_node: Node
    ) -> Optional[ChildClass]:
        """
        Create instance and initialize from database.

        :returns: Instance if init from database was possible or None.
                  None means there was no data in the database to initialize this object.
        :raises SerializeError: If database incompatible.
        """
        self = super().__new__(cls)
        if await self.ainit_from_database(database, db_node):
            return self
        else:
            return None

    @abstractmethod
    async def ainit(self) -> None:
        pass

    @abstractmethod
    async def ainit_from_database(self, database: Database, db_node: Node) -> bool:
        """
        Initialize from database.

        :returns: If init from database was possible.
                  False means there was no data in the database to initialize this object.
        :raises SerializeError: If database incompatible.
        """
        pass
