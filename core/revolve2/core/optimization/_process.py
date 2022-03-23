from typing import Any, Optional, Type, TypeVar

from sqlalchemy.ext.asyncio import AsyncEngine
from sqlalchemy.ext.asyncio.session import AsyncSession

TChild = TypeVar("TChild", bound="Process")


# TODO This should be a decorator to allow for proper typing
# but this is not possible because typing does not include a 'Compositite'
# counterpart of 'Union' so the return type of the decorator cannot be set
# to be a combination of Process and the original class.
# So for now the user must inherit from Process.
# However the ainit functions cannot be made abstractmethod because the arguments
# are not known in advance(they could be retrieved using a decorator though), so
# that would violate lizkov's substitution principle.
# So, to get proper typing (in python 3.8) the user must simply remember to implement
# these two functions.
class Process:
    def __init__(self) -> None:
        raise RuntimeError("Call 'new' or 'from_database' instead.")

    # Implement the following functions:

    # @abstractmethod
    # async def ainit_new(
    #    self,
    #    database: Database,
    #    session: AsyncSession,
    #    process_id: int,
    #    *args: Any,
    #    **kwargs: Any
    # ) -> None:
    #    """
    #    Init called when creating an instance through 'new'.
    #    Use the session to save initial settings that could be loaded using `ainit_from_database`.
    #    The session must not be commit, but it may be flushed.
    #    """
    #    pass

    # @abstractmethod
    # async def ainit_from_database(
    #     self,
    #     database: Database,
    #     session: AsyncSession,
    #     process_id: int,
    #     *args: Any,
    #     **kwargs: Any
    # ) -> bool:
    #     """
    #     Init called when creating an instance through 'from_database'.
    #     """
    #     pass

    @classmethod
    async def new(
        cls: Type[TChild],
        database: AsyncEngine,
        process_id: int,
        *args: Any,
        **kwargs: Any
    ) -> TChild:
        self = super().__new__(cls)
        async with AsyncSession(database) as session:
            async with session.begin():
                await self.ainit_new(database, session, process_id, *args, **kwargs)  # type: ignore # must have this function, see above
        return self

    @classmethod
    async def from_database(
        cls: Type[TChild],
        database: AsyncEngine,
        process_id: int,
        *args: Any,
        **kwargs: Any
    ) -> Optional[TChild]:
        """
        Create instance and initialize from database.
        :returns: Instance if init from database was possible or None.
                  None means there was no data in the database to initialize this object.
        :raises SerializeError: If database incompatible.
        """
        self = super().__new__(cls)
        async with AsyncSession(database) as session:
            if await self.ainit_from_database(  # type: ignore # must have this function, see above
                database, session, process_id, *args, **kwargs
            ):
                return self
            else:
                return None
