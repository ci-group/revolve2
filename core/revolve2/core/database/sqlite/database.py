from __future__ import annotations

import contextlib
import os

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

from ..database import Database as DatabaseBase
from ..node import Node
from .node_impl import NodeImpl
from .schema import Base, DbNode
from .transaction import Transaction


class Database(DatabaseBase):
    __create_key = object()

    _db_sessionmaker: sessionmaker

    def __init__(self, create_key) -> None:
        if create_key is not self.__create_key:
            raise ValueError(
                "Sqlite database can only be created through its factory function."
            )

    async def create(root_directory: str) -> Database:
        self = Database(Database.__create_key)

        if not os.path.isdir(root_directory):
            os.makedirs(root_directory, exist_ok=True)

        engine = create_engine(f"sqlite:///{root_directory}/db.sqlite")
        Base.metadata.create_all(engine)
        self._db_sessionmaker = sessionmaker(bind=engine)

        self._create_root_node()

        return self

    @contextlib.contextmanager
    def begin_transaction(self) -> Transaction:
        session = self._db_sessionmaker()
        yield Transaction(session)
        session.commit()
        session.close()

    @property
    def root(self) -> Node:
        return Node(NodeImpl(0))

    def _create_root_node(self) -> None:

        with self.begin_transaction() as ses:
            print(ses._session.query(DbNode).count())
            if ses._session.query(DbNode).count() == 0:
                ses._session.add(DbNode(0, None, 0))
