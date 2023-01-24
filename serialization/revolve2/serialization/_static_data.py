from __future__ import annotations

from typing import Any, Dict, List, Union

from typing_extensions import TypeGuard

StaticData = Union[
    List["StaticData"], Dict[str, "StaticData"], None, bool, int, float, str, bytes
]


def is_static_data(to_check: Any) -> TypeGuard[StaticData]:
    """
    Check if the provided object is `StaticData`.

    :param to_check: The data to check.
    :returns: If the provided object is `StaticData`.
    """
    if (
        to_check is None
        or type(to_check) == bool
        or type(to_check) == int
        or type(to_check) == float
        or type(to_check) == str
        or type(to_check) == bytes
    ):
        return True
    elif type(to_check) == list:
        return all([is_static_data(child) for child in to_check])
    elif type(to_check) == dict:
        return all(
            [type(key) == str and is_static_data(val) for key, val in to_check.items()]
        )
    else:
        return False
