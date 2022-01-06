from abc import ABC, abstractmethod

from .node import Node
from .transaction import Transaction


class Database(ABC):
    @abstractmethod
    def begin_transaction(self) -> Transaction:
        """
        Begin a transaction context.
        """
        pass

    @property
    @abstractmethod
    def root(self) -> Node:
        """
        Get the root node of the database.
        """
        pass
