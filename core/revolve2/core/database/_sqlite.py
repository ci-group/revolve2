import os

from sqlalchemy import create_engine
from sqlalchemy.engine import Engine
from sqlalchemy.ext.asyncio import AsyncEngine, create_async_engine


def open_async_database_sqlite(
    db_root_directory: str, create: bool = False
) -> AsyncEngine:
    """
    Open an SQLAlchemy SQLite async database.

    :param db_root_directory: Directory to store/load the database in/from.
    :param create: If true, database will be created. If false, error will be raised.
    :returns: The opened database.
    :raises RuntimeError: In case the database doesn't exist but `create` is False
    """
    dbpath = f"{db_root_directory}/db.sqlite"
    dburl = f"sqlite+aiosqlite:///{dbpath}"

    if not create and not os.path.exists(dbpath):
        raise RuntimeError(f"Trying to open database but it does not exist: {dbpath}")

    os.makedirs(db_root_directory, exist_ok=True)
    return create_async_engine(dburl)


def open_database_sqlite(db_root_directory: str, create: bool = False) -> Engine:
    """
    Open an SQLAlchemy SQLite database.

    :param db_root_directory: Directory to store/load the database in/from.
    :param create: If true, database will be created. If false, error will be raised.
    :returns: The opened database.
    :raises RuntimeError: In case the database doesn't exist but `create` is False
    """
    dbpath = f"{db_root_directory}/db.sqlite"
    dburl = f"sqlite:///{dbpath}"

    if not create and not os.path.exists(dbpath):
        raise RuntimeError(f"Trying to open database but it does not exist: {dbpath}")

    os.makedirs(db_root_directory, exist_ok=True)
    return create_engine(dburl)
