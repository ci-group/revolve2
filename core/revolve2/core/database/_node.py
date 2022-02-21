from __future__ import annotations

from typing import Optional, Union

from ._database_error import DatabaseError
from ._db_data import DbData
from ._node_impl import NodeImpl
from ._transaction import Transaction
from ._uninitialized import Uninitialized


class Node:
    """
    Represents a node in a database.
    Can be either DbData or unitialized.
    """

    _impl: Optional[NodeImpl]

    def __init__(self, impl: Optional[NodeImpl] = None):
        self._impl = impl

    @property
    def is_stub(self) -> bool:
        """
        If node is not yet linked to the database but a stub created by the user.
        """
        return self._impl is None

    def get_db_data(self, txn: Transaction) -> Union[DbData, Uninitialized]:
        """
        Read the underlying DbData from the database.
        """
        if self._impl is None:
            raise DatabaseError()

        return self._impl.get_db_data(txn)

    def set_db_data(self, txn: Transaction, db_data: DbData) -> None:
        """
        Set the underlying db_data in the database.
        If db_data is not uninitialized, raises DatabaseError.
        """
        if self._impl is None:
            raise DatabaseError(
                "Node not usable yet. It is a stub created by the user that has not yet been linked with the database."
            )

        self._impl.set_db_data(txn, db_data)

    def _set_impl(self, impl: NodeImpl) -> None:
        self._impl = impl
