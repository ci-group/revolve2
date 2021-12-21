import base64
import json
import re
from json.decoder import JSONDecodeError
from typing import Any, Dict, List, Tuple, Union

from ..database_error import DatabaseError
from ..list import List as ListIface
from ..node import Node as NodeIface
from ..node_impl import NodeImpl as NodeImplBase
from ..object import Object, is_object
from ..transaction import Transaction as TransactionBase
from ..uninitialized import Uninitialized
from .list_impl import List as ListImpl
from .schema import DbNode
from .transaction import Transaction


class NodeImpl(NodeImplBase):
    _id: int

    def __init__(self, id: int) -> None:
        self._id = id

    def get_object(self, txn: TransactionBase) -> Union[Object, Uninitialized]:
        assert isinstance(txn, Transaction)

        row = self._get_row(txn)

        if row.type == 0:
            return Uninitialized()
        elif row.type == 1:
            object = row.object
            if not isinstance(object, str):
                raise DatabaseError("Object of Node should be string, but it is not.")

            try:
                return json.loads(object, cls=_JSONDecoder)
            except JSONDecodeError as err:
                raise DatabaseError("Object of Node is not valid JSON.")
        elif row.type == 2:
            return ListIface(ListImpl(row.id))
        else:
            raise DatabaseError("Database corrupted. Unexpected type.")

    def set_object(self, txn: TransactionBase, object: Object) -> None:
        assert isinstance(txn, Transaction)

        if not is_object(object):
            raise DatabaseError("Not a valid object.")

        dbrow = self._get_row(txn)
        if dbrow.object is not None or dbrow.type != 0:
            raise DatabaseError("Node object already set. Cannot overwrite.")

        node_stubs, list_stubs = self._find_node_stubs(object)

        if not all([stub.is_stub for stub in node_stubs]):
            raise DatabaseError(
                "Cannot add a node to an object that is not a stub. You are reusing nodes."
            )

        dbnodes = [DbNode(0, None) for _ in node_stubs]
        dblistnodes = [DbNode(2, None) for _ in list_stubs]

        for node in dbnodes:
            txn._session.add(node)
        for node in dblistnodes:
            txn._session.add(node)

        txn._session.flush()

        for node_stub, dbnode in zip(node_stubs, dbnodes):
            node_stub._set_impl(NodeImpl(dbnode.id))
        for list_stub, dblistnode in zip(list_stubs, dblistnodes):
            list_stub._set_impl(ListImpl(dblistnode.id))

        jsoned = json.dumps(object, cls=_JSONEncoder)
        dbrow.object = jsoned
        dbrow.type = 1

    def _get_row(self, txn: Transaction) -> Any:  # TODO sqlalchemy typing
        query = txn._session.query(DbNode).filter(DbNode.id == self._id)
        if query.count() != 1:
            raise DatabaseError(
                "Database out of sync with program. Node does not exist in database but it should."
            )
        return query.first()

    @classmethod
    def _find_node_stubs(
        cls, object: Object
    ) -> Tuple[List[NodeIface], List[ListIface]]:
        if isinstance(object, NodeIface):
            return [object], []
        elif isinstance(object, ListIface):
            return [], [object]
        elif isinstance(object, list):
            nodes = []
            lists = []
            for child in object:
                new_nodes, new_lists = cls._find_node_stubs(child)
                nodes += new_nodes
                lists += new_lists
            return (nodes, lists)
        elif isinstance(object, dict):
            nodes = []
            lists = []
            for child in object.values():
                new_nodes, new_lists = cls._find_node_stubs(child)
                nodes += new_nodes
                lists += new_lists
            return (nodes, lists)
        else:
            return ([], [])


class _JSONEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, NodeIface):
            assert isinstance(obj._impl, NodeImpl)
            return f"__node_{obj._impl._id}__"
        elif isinstance(obj, ListIface):
            assert isinstance(obj._impl, ListImpl)
            return f"__node_{obj._impl._id}__"
        elif isinstance(obj, bytes):
            return f"__bytes_{base64.b64encode(obj).decode('ascii')}"
        else:
            return json.JSONEncoder.default(self, obj)


class _JSONDecoder(json.JSONDecoder):
    def __init__(self, *args, **kwargs):
        json.JSONDecoder.__init__(self, object_hook=self.object_hook, *args, **kwargs)

    def object_hook(self, dct: Dict):
        res = {}
        for key, val in dct.items():
            if (
                isinstance(val, str)
                and (reres := re.search(r"^__node_(\d+)__$", val)) is not None
            ):
                id = int(reres.group(1))
                res[key] = NodeIface(NodeImpl(id))
            elif (
                isinstance(val, str)
                # this crazy regex searches for __bytes_<base64>__ and sets the base64 to group(1)
                # https://stackoverflow.com/questions/475074/regex-to-parse-or-validate-base64-data
                and (
                    reres := re.search(
                        r"^__bytes_((?:[A-Za-z0-9+/]{4})*(?:[A-Za-z0-9+/]{2}==|[A-Za-z0-9+/]{3}=)?)__$",
                        val,
                    )
                )
                is not None
            ):
                bytes = base64.decode(reres.group(1))
                res[key] = bytes
            else:
                res[key] = val

        return res
