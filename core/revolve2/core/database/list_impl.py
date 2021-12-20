from abc import ABC, abstractmethod

from .node import Node
from .transaction import Transaction


class ListImpl(ABC):
    @abstractmethod
    def get_or_append(self, txn: Transaction, index: int) -> Node:
        pass
