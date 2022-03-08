from __future__ import annotations

from typing import Any

from revolve2.serialization import StaticData, is_static_data

from ._db_data import DbData, is_db_data
from ._list import List as DbList
from ._node import Node


def dynamic_cast_db_data(data: Any) -> DbData:
    """
    Check if data is DbData, returning data cast.
    :raises TypeError: If data is not DbData.
    """

    if not is_db_data(data):
        raise TypeError("Data to be cast is not DbData")
    return data


def dynamic_cast_static_data(data: Any) -> StaticData:
    """
    Check if data is StaticData, returning data cast.
    :raises TypeError: If data is not StaticData.
    """

    if not is_static_data(data):
        raise TypeError("Data to be cast is not StaticData")
    return data


def dynamic_cast_int(data: Any) -> int:
    """
    Check if data is int, returning data cast.
    :raises TypeError: If data is not int.
    """

    if type(data) != int:
        raise TypeError("Data to be cast is not int")
    return data


def dynamic_cast_float(data: Any) -> float:
    """
    Check if data is float, returning data cast.
    :raises TypeError: If data is not float.
    """

    if type(data) != float:
        raise TypeError("Data to be cast is not float")
    return data


def dynamic_cast_str(data: Any) -> str:
    """
    Check if data is str, returning data cast.
    :raises TypeError: If data is not str.
    """

    if type(data) != str:
        raise TypeError("Data to be cast is not str")
    return data


def dynamic_cast_bytes(data: Any) -> bytes:
    """
    Check if data is bytes, returning data cast.
    :raises TypeError: If data is not bytes.
    """

    if type(data) != bytes:
        raise TypeError("Data to be cast is not bytes")
    return data


def dynamic_cast_dict(data: Any) -> dict:
    """
    Check if data is dict, returning data cast.
    :raises TypeError: If data is not dict.
    """

    if type(data) != dict:
        raise TypeError("Data to be cast is not dict")
    return data


def dynamic_cast_list(data: Any) -> list:
    """
    Check if data is list, returning data cast.
    :raises TypeError: If data is not list.
    """

    if type(data) != list:
        raise TypeError("Data to be cast is not list")
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
