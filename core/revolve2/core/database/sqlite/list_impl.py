from ..list_impl import ListImpl as ListImplBase
from ..node import Node
from ..transaction import Transaction as TransactionBase
from .transaction import Transaction


class List(ListImplBase):
    _id: int

    def __init__(self, id: int) -> None:
        self._id = id

    def get_or_append(self, txn: TransactionBase, index: int) -> Node:
        assert isinstance(txn, Transaction)
        raise NotImplementedError()
