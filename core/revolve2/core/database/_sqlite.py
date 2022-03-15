from sqlalchemy.ext.asyncio import create_async_engine
from ._database import Database
import os


def open_database_sqlite(db_root_directory) -> Database:
    os.makedirs(db_root_directory, exist_ok=True)
    return Database(
        create_async_engine(f"sqlite+aiosqlite:///{db_root_directory}/db.sqlite")
    )
