import sqlalchemy.ext.asyncio
import sqlalchemy.ext.asyncio.engine
import sqlalchemy.orm


class Database:
    _engine: sqlalchemy.ext.asyncio.engine.AsyncEngine
    _session_maker: sqlalchemy.orm.sessionmaker

    def __init__(self, engine: sqlalchemy.ext.asyncio.engine.AsyncEngine) -> None:
        self._engine = engine
        self._session_maker = sqlalchemy.orm.sessionmaker(
            self._engine,
            expire_on_commit=False,
            class_=sqlalchemy.ext.asyncio.AsyncSession,
        )

    @property
    def engine(self) -> sqlalchemy.ext.asyncio.engine.AsyncEngine:
        return self._engine

    def session(self) -> sqlalchemy.ext.asyncio.AsyncSession:
        return self._session_maker()
