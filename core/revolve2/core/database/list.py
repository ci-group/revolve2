from abc import ABC, abstractmethod

from .node import Node
from .transaction import Transaction


class List(ABC):
    @abstractmethod
    @property
    def is_stub(self) -> bool:
        """
        If node is not yet linked to the database but a stub created by the user.
        """
        pass

    @abstractmethod
    def get_or_append(self, tnx: Transaction, index: int) -> Node:
        """
        Get the item at the given index, or if the list is one short,
        append to the list and return the new node.
        If the list is more than one too short or the index is not the last index in the list,
        DatabaseError is thrown.
        """
        pass
