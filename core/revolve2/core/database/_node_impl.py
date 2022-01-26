from abc import ABC, abstractmethod
from typing import Union

from ._object import Object
from ._transaction import Transaction
from ._uninitialized import Uninitialized


class NodeImpl(ABC):
    @abstractmethod
    def get_object(self, txn: Transaction) -> Union[Object, Uninitialized]:
        pass

    @abstractmethod
    def set_object(self, txn: Transaction, object: Object) -> None:
        pass
