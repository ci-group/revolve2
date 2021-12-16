from typing import Dict, List, Union

from .list import List as DbList
from .node import Node

Object = Union[
    List["Object"], Dict[str, "Object"], str, float, int, bytes, None, Node, DbList
]
