from typing import Any, Dict, List, Union, cast

StaticData = Union[
    List["StaticData"], Dict[str, "StaticData"], str, float, int, bytes, None
]


def is_data(to_check: Any) -> bool:
    if (
        to_check is None
        or type(to_check) == int
        or type(to_check) == float
        or type(to_check) == str
        or type(to_check) == bytes
    ):
        return True
    elif type(to_check) == list:
        return all([is_data(child) for child in to_check])
    elif type(to_check) == dict:
        return all(
            [
                type(key) == str and is_data(val)
                for key, val in cast(Dict[Any, Any], to_check).items()
            ]
        )
    else:
        return False
