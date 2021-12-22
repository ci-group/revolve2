from typing import TYPE_CHECKING, Any, Dict, List, Union

from .static_data import StaticData, cast, is_static_data

if TYPE_CHECKING:
    from .list import List as DbList
    from .node import Node

Object = Union[List["Object"], Dict[str, "Object"], StaticData, "Node", "DbList"]


def is_object(to_check: Any) -> bool:
    from .list import List as DbList
    from .node import Node

    if isinstance(to_check, DbList):
        return True
    elif isinstance(to_check, Node):
        return True
    elif isinstance(to_check, list):
        return all([is_object(child) for child in to_check])
    elif isinstance(to_check, dict):
        return all(
            [
                isinstance(key, str) and is_object(val)
                for key, val in cast(Dict[Any, Any], to_check).items()
            ]
        )
    elif is_static_data(to_check):
        return True
    else:
        return False
