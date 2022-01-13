from typing import Any

from .object import Object, is_object
from .static_data import StaticData, is_static_data
from .node import Node
from .list import List as DbList


def dynamic_cast_object(data: Any) -> Object:
    """
    Check if data is Object, returning data cast.
    :raises TypeError: If data is not Object.
    """

    if not is_object(data):
        raise TypeError("Data to be cast is not Object")
    return data


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


def dynamic_cast_node(data: Any) -> Node:
    """
    Check if data is Node, returning data cast.
    :raises TypeError: If data is not Node.
    """

    if not isinstance(data, Node):
        raise TypeError("Data to be cast is not Node")
    return data


def dynamic_cast_dblist(data: Any) -> DbList:
    """
    Check if data is DbList, returning data cast.
    :raises TypeError: If data is not DbList.
    """

    if not isinstance(data, DbList):
        raise TypeError("Data to be cast is not DbList")
    return data
