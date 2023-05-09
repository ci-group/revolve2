import os
from pathlib import Path

from sqlalchemy import create_engine
from sqlalchemy.engine import Engine
from sqlalchemy.ext.asyncio import AsyncEngine, create_async_engine

from ._open_check import OpenCheck


def open_async_database_sqlite(
    db_file: str, open_check: OpenCheck = OpenCheck.OPEN_IF_EXISTS
) -> AsyncEngine:
    """
    Open an SQLAlchemy SQLite async database.

    :param db_file: File for the database.
    :param open_check: Performs checks when opening the database according to the parameter value.
    :returns: The opened database.
    :raises RuntimeError: In case the database does not exist but `create` is False.
    """
    __common(db_file, open_check)
    return create_async_engine(f"sqlite+aiosqlite:///{db_file}")


def open_database_sqlite(
    db_file: str, open_check: OpenCheck = OpenCheck.OPEN_IF_EXISTS
) -> Engine:
    """
    Open an SQLAlchemy SQLite database.

    :param db_file: File for the database.
    :param open_check: Performs checks when opening the database according to the parameter value.
    :returns: The opened database.
    :raises RuntimeError: In case the database does not exist but `create` is False.
    """
    __common(db_file, open_check)
    return create_engine(f"sqlite:///{db_file}")


def __common(db_file: str, open_check: OpenCheck = OpenCheck.OPEN_IF_EXISTS) -> None:
    exists = os.path.exists(db_file)
    if open_check == OpenCheck.OPEN_IF_EXISTS:
        if not exists:
            raise RuntimeError(
                f"Open check set to OPEN_IF_EXISTS and database does not exist: {db_file}"
            )
    elif open_check == OpenCheck.OPEN_OR_CREATE:
        os.makedirs(Path(db_file).parent, exist_ok=True)
    elif open_check == OpenCheck.NOT_EXISTS_AND_CREATE:
        if exists:
            raise RuntimeError(
                f"Open check set to NOT_EXISTS_AND_CREATE and database exists: {db_file}"
            )
        os.makedirs(Path(db_file).parent, exist_ok=True)
    elif open_check == OpenCheck.OVERWITE_IF_EXISTS:
        if exists:
            os.remove(db_file)
        else:
            os.makedirs(Path(db_file).parent, exist_ok=True)
