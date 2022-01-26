from revolve2.core.database import DatabaseError

from .._list_impl import ListImpl as ListImplBase
from .._node import Node
from .._transaction import Transaction as TransactionBase
from ._schema import DbListItem, DbNode
from ._transaction import Transaction


class List(ListImplBase):
    _id: int

    def __init__(self, id: int) -> None:
        self._id = id

    def get_or_append(self, txn: TransactionBase, index: int) -> Node:
        from ._node_impl import NodeImpl

        assert isinstance(txn, Transaction)
        assert index >= 0

        allitems = txn._session.query(DbListItem).filter(
            DbListItem.list_node_id == self._id
        )
        if index > allitems.count():
            raise DatabaseError(
                "Requesting to get or append index in list that is more than one out of bounds."
            )
        elif index == allitems.count():
            child = DbNode(0, None)
            txn._session.add(child)
            txn._session.flush()
            txn._session.add(DbListItem(index, self._id, child.id))
            return Node(NodeImpl(child.id))
        else:
            item = allitems.filter(DbListItem.index == index).first()
            if item is None:
                raise DatabaseError(
                    "Database corrupt. Indeices of nodes not in line with number of nodes."
                )
            return Node(NodeImpl(item.child_node_id))

    def append(self, txn: TransactionBase) -> Node:
        from ._node_impl import NodeImpl

        assert isinstance(txn, Transaction)

        index = (
            txn._session.query(DbListItem)
            .filter(DbListItem.list_node_id == self._id)
            .count()
        )

        child = DbNode(0, None)
        txn._session.add(child)
        txn._session.flush()
        txn._session.add(DbListItem(index, self._id, child.id))
        return Node(NodeImpl(child.id))

    def get(self, txn: TransactionBase, index: int) -> Node:
        from ._node_impl import NodeImpl

        assert isinstance(txn, Transaction)

        item = txn._session.query(DbListItem).filter(
            DbListItem.list_node_id == self._id, DbListItem.index == index
        )
        if item.count() != 1:
            raise DatabaseError("Index out of bounds or database corrupted.")
        first = item.first()
        assert first is not None  # cannot happen but mypy doesn't recognize
        return Node(NodeImpl(first.child_node_id))

    def len(self, txn: TransactionBase) -> int:
        assert isinstance(txn, Transaction)

        return (
            txn._session.query(DbListItem)
            .filter(DbListItem.list_node_id == self._id)
            .count()
        )
