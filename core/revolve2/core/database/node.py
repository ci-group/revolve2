from typing import Optional, Union

from .database_error import DatabaseError
from .node_impl import NodeImpl
from .object import Object
from .transaction import Transaction
from .uninitialized import Uninitialized


class Node:
    """
    Represents a node in a database.
    Can be either an object or unitialized.
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

    def get_object(self, txn: Transaction) -> Union[Object, Uninitialized]:
        """
        Read the underlying object from the database.
        """
        if self._impl is None:
            raise DatabaseError()

        return self._impl.get_object(txn)

    def set_object(self, txn: Transaction, object: Object) -> None:
        """
        Set the underlying object in the database.
        If object is not uninitialized, raises DatabaseError.
        """
        if self._impl is None:
            raise DatabaseError(
                "Node not usable yet. It is a stub created by the user that has not yet been linked with the database."
            )

        self._impl.set_object(txn, object)

    def _set_impl(self, impl: NodeImpl) -> None:
        self._impl = impl
