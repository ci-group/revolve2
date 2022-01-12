from typing import Any, Dict, List, Type, Union

from typing_extensions import TypeGuard

StaticData = Union[  # type: ignore # TODO this is not yet supported by mypy
    List["StaticData"], Dict[str, "StaticData"], str, float, int, bytes, None  # type: ignore
]


def is_static_data(to_check: Any) -> TypeGuard[StaticData]:
    if (
        to_check is None
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


def dynamic_cast_static_data(data: Any) -> StaticData:
    """
    Check if data is StaticData, returning data cast.
    :raises TypeError: If data is not StaticData.
    """

    if not is_static_data(data):
        raise TypeError("Data to be cast is not StaticData")
    return data


def dynamic_cast_bytes(data: Any) -> bytes:
    """
    Check if data is bytes, returning data cast.
    :raises TypeError: If data is not bytes.
    """

    if type(data) != bytes:
        raise TypeError("Data to be cast is not bytes")
    return data


def dynamic_cast_float(data: Any) -> float:
    """
    Check if data is float, returning data cast.
    :raises TypeError: If data is not float.
    """

    if type(data) != float:
        raise TypeError("Data to be cast is not float")
    return data
