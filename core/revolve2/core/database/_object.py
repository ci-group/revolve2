from __future__ import annotations
from typing import TYPE_CHECKING, Any, Dict, List, Union

from typing_extensions import TypeGuard

from ._static_data import StaticData, is_static_data

if TYPE_CHECKING:
    from ._list import List as DbList
    from ._node import Node

Object = Union[List["Object"], Dict[str, "Object"], StaticData, "Node", "DbList"]  # type: ignore # TODO this is not yet supported by mypy


def is_object(to_check: Any) -> TypeGuard[Object]:
    from ._list import List as DbList
    from ._node import Node

    if isinstance(to_check, DbList):
        return True
    elif isinstance(to_check, Node):
        return True
    elif isinstance(to_check, list):
        return all([is_object(child) for child in to_check])
    elif isinstance(to_check, dict):
        return all(
            [isinstance(key, str) and is_object(val) for key, val in to_check.items()]
        )
    elif is_static_data(to_check):
        return True
    else:
        return False
