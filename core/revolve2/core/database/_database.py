import sqlalchemy.ext.asyncio
import sqlalchemy.ext.asyncio.engine
import sqlalchemy.orm
from sqlalchemy.ext.asyncio.session import AsyncSession
from typing import TYPE_CHECKING
from sqlalchemy.orm import sessionmaker

# there is a bug in sqlalchemy
# typing does not match runtime
# https://github.com/sqlalchemy/sqlalchemy/issues/7656
if TYPE_CHECKING:
    TSession = sessionmaker[AsyncSession]
else:
    # anything that doesn't raise an exception
    TSession = sessionmaker


class Database:
    _engine: sqlalchemy.ext.asyncio.engine.AsyncEngine
    _session_maker: TSession

    def __init__(self, engine: sqlalchemy.ext.asyncio.engine.AsyncEngine) -> None:
        self._engine = engine
        self._session_maker = sessionmaker(
            self._engine,
            expire_on_commit=False,
            class_=sqlalchemy.ext.asyncio.AsyncSession,
        )

    @property
    def engine(self) -> sqlalchemy.ext.asyncio.engine.AsyncEngine:
        return self._engine

    def session(self) -> sqlalchemy.ext.asyncio.AsyncSession:
        return self._session_maker()
