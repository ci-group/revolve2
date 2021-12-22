from abc import ABC, abstractmethod

from .node import Node
from .transaction import Transaction


class ListImpl(ABC):
    @abstractmethod
    def get_or_append(self, txn: Transaction, index: int) -> Node:
        pass

    @abstractmethod
    def append(self, txn: Transaction) -> Node:
        pass

    @abstractmethod
    def get(self, txn: Transaction, index: int) -> Node:
        pass

    @abstractmethod
    def len(self, txn: Transaction) -> int:
        pass
