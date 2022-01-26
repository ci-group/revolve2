from __future__ import annotations

import os

from sqlalchemy import create_engine
from sqlalchemy.orm import Session, sessionmaker

from .._database import Database as DatabaseBase
from .._node import Node
from .._transaction import Transaction as TransactionBase
from ._node_impl import NodeImpl
from ._schema import Base, DbNode
from ._transaction import Transaction


class Database(DatabaseBase):
    __create_key = object()

    _db_sessionmaker: sessionmaker

    def __init__(self, create_key: object) -> None:
        if create_key is not self.__create_key:
            raise ValueError(
                "Sqlite database can only be created through its factory function."
            )

    @classmethod
    async def create(cls, root_directory: str) -> Database:
        self = cls(Database.__create_key)

        if not os.path.isdir(root_directory):
            os.makedirs(root_directory, exist_ok=True)

        engine = create_engine(f"sqlite:///{root_directory}/db.sqlite")
        Base.metadata.create_all(engine)
        self._db_sessionmaker = sessionmaker(bind=engine)

        self._create_root_node()

        return self

    def begin_transaction(self) -> TransactionBase:
        session: Session = self._db_sessionmaker()
        return Transaction(session)

    @property
    def root(self) -> Node:
        return Node(NodeImpl(0))

    def _create_root_node(self) -> None:

        with self.begin_transaction() as ses:
            assert isinstance(ses, Transaction)
            if ses._session.query(DbNode).count() == 0:
                ses._session.add(DbNode(0, None, 0))
