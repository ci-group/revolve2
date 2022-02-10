from __future__ import annotations
from typing import TYPE_CHECKING, Any, Dict, List, Union

from typing_extensions import TypeGuard

from revolve2.serialization import StaticData, is_static_data

if TYPE_CHECKING:
    from ._list import List as DbList
    from ._node import Node

DbData = Union[List["DbData"], Dict[str, "DbData"], StaticData, "Node", "DbList"]  # type: ignore # TODO this is not yet supported by mypy


def is_db_data(to_check: Any) -> TypeGuard[DbData]:
    from ._list import List as DbList
    from ._node import Node

    if isinstance(to_check, DbList):
        return True
    elif isinstance(to_check, Node):
        return True
    elif isinstance(to_check, list):
        return all([is_db_data(child) for child in to_check])
    elif isinstance(to_check, dict):
        return all(
            [isinstance(key, str) and is_db_data(val) for key, val in to_check.items()]
        )
    elif is_static_data(to_check):
        return True
    else:
        return False
