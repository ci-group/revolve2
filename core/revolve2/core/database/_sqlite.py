import os

from sqlalchemy import create_engine
from sqlalchemy.engine import Engine
from sqlalchemy.ext.asyncio import AsyncEngine, create_async_engine


def open_async_database_sqlite(db_root_directory: str) -> AsyncEngine:
    os.makedirs(db_root_directory, exist_ok=True)
    return create_async_engine(f"sqlite+aiosqlite:///{db_root_directory}/db.sqlite")


def open_database_sqlite(db_root_directory: str) -> Engine:
    os.makedirs(db_root_directory, exist_ok=True)
    return create_engine(f"sqlite:///{db_root_directory}/db.sqlite")
