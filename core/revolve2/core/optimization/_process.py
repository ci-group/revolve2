from typing import Any, Optional, Type, TypeVar

from sqlalchemy.ext.asyncio import AsyncEngine
from sqlalchemy.ext.asyncio.session import AsyncSession

TChild = TypeVar("TChild", bound="Process")


class Process:
    """
    Provides asynchronous instantiation of the class that inherits from this.

    The inherited class must implement the `ainit_new` and `ainit_from_database` functions,
    as specified in the comments below.

    Instantiate either using `new` or `from_database`.
    In the latter case the inherited class must attempt to load its state from the provided database.

    TODO This should be a decorator to allow for proper typing
    but this is not possible because typing does not include a 'Compositite'
    counterpart of 'Union' so the return type of the decorator cannot be set
    to be a combination of Process and the original class.
    So for now the user must inherit from Process.
    However the ainit functions cannot be made abstractmethod because the arguments
    are not known in advance(they could be retrieved using a decorator though), so
    that would violate lizkov's substitution principle.
    So, to get proper typing (in python 3.8) the user must simply remember to implement
    these two functions.
    """

    def __init__(self) -> None:
        """
        Do not call this function.

        Instantiate using `new` or `from_database` instead.
        """
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
    #    Initialize this object.
    #
    #    Called when creating an instance through 'new'.
    #    Use the session to save initial settings that could be loaded using `ainit_from_database`.
    #    The session must not be commit, but it may be flushed.
    #
    #    :param database: Database to use for this optimizer.
    #    :param session: Session to use when saving data to the database during initialization.
    #    :param process_id: Unique identifier in the completely program specifically made for this optimizer.
    #    :param args: Other positional arguments required for the inheriting class.
    #    :param kwargs: Other named arguments required for the inheriting class.
    #    """

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
    #     Initialize this object.
    #
    #     Called when creating an instance through 'from_database'.
    #
    #    :param database: Database to use for this optimizer.
    #    :param session: Session to use when loading and saving data to the database during initialization.
    #    :param process_id: Unique identifier in the completely program specifically made for this optimizer.
    #    :param args: Other positional arguments required for the inheriting class.
    #    :param kwargs: Other named arguments required for the inheriting class.
    #    :returns: True if this complete object could be deserialized from the database.
    #    :raises IncompatibleError: In case the database is not compatible with this class.
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
        """
        Create a new instance of this class.

        It will be initialized using `ainit_new`.

        :param database: Database to use for this optimizer.
        :param session: Session to use when saving data to the database during initialization.
        :param args: Other positional arguments required for the inheriting class.
        :param kwargs: Other named arguments required for the inheriting class.
        :returns: An instance of this class.
        """
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
        Create a new instance of this class.

        It will be initialized using `ainit_from_database`.

        :param database: Database to use for this optimizer.
        :param process_id: Unique identifier in the completely program specifically made for this optimizer.
        :param args: Other positional arguments required for the inheriting class.
        :param kwargs: Other named arguments required for the inheriting class.
        :returns: An instance of this class or None if the class could not be completely deserialized.
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
