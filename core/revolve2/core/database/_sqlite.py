import os

import sqlalchemy
from sqlalchemy.engine import Engine
from sqlalchemy.ext.asyncio import create_async_engine

from ._database import Database


def open_database_sqlite(db_root_directory) -> Database:
    os.makedirs(db_root_directory, exist_ok=True)
    return Database(
        create_async_engine(f"sqlite+aiosqlite:///{db_root_directory}/db.sqlite")
    )


def create_sync_engine_sqlite(db_root_directory) -> Engine:
    return sqlalchemy.create_engine(f"sqlite:///{db_root_directory}/db.sqlite")
