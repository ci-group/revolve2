from abc import ABC, abstractmethod
from typing import Union

from ._db_data import DbData
from ._transaction import Transaction
from ._uninitialized import Uninitialized


class NodeImpl(ABC):
    @abstractmethod
    def get_db_data(self, txn: Transaction) -> Union[DbData, Uninitialized]:
        pass

    @abstractmethod
    def set_db_data(self, txn: Transaction, db_data: DbData) -> None:
        pass
