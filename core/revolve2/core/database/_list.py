from typing import Optional

from ._database_error import DatabaseError
from ._list_impl import ListImpl
from ._node import Node
from ._transaction import Transaction


class List:
    _impl: Optional[ListImpl]

    def __init__(self, impl: Optional[ListImpl] = None):
        self._impl = impl

    @property
    def is_stub(self) -> bool:
        """
        If list is not yet linked to the database but a stub created by the user.
        """
        return self._impl is None

    def get_or_append(self, txn: Transaction, index: int) -> Node:
        """
        Get the item at the given index, or if the list is one short,
        append to the list and return the new node.
        If the list is more than one too short or the index is not the last index in the list,
        DatabaseError is thrown.
        """
        if self._impl is None:
            raise DatabaseError(
                "List not usable yet. It is a stub created by the user that has not yet been linked with the database."
            )

        return self._impl.get_or_append(txn, index)

    def append(self, txn: Transaction) -> Node:
        """
        Append a new node to the list and return it.
        """
        if self._impl is None:
            raise DatabaseError(
                "List not usable yet. It is a stub created by the user that has not yet been linked with the database."
            )

        return self._impl.append(txn)

    def get(self, txn: Transaction, index: int) -> Node:
        """
        Get the item at the given index.
        :raises: DatabaseError if out of bounds
        """
        if self._impl is None:
            raise DatabaseError(
                "List not usable yet. It is a stub created by the user that has not yet been linked with the database."
            )

        return self._impl.get(txn, index)

    def len(self, txn: Transaction) -> int:
        """
        Get the length of the list.
        """
        if self._impl is None:
            raise DatabaseError(
                "List not usable yet. It is a stub created by the user that has not yet been linked with the database."
            )

        return self._impl.len(txn)

    def _set_impl(self, impl: ListImpl) -> None:
        self._impl = impl
