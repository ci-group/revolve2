from __future__ import annotations
from typing import Any, Type, TypeVar, Union

from ._static_data import (
    StaticData,
    is_static_data,
)
from ._serializable import Serializable
from ._serialize_error import SerializeError


def serialize(to_serialize: Union[Serializable, StaticData]) -> StaticData:
    if isinstance(to_serialize, Serializable):
        return to_serialize.serialize()
    elif is_static_data(to_serialize):
        return to_serialize
    else:
        raise SerializeError()


T = TypeVar("T", Serializable, StaticData)


def deserialize(data: StaticData, as_type: Type[T]) -> T:
    if issubclass(as_type, Serializable):
        return as_type.deserialize(data)
    elif (
        (as_type is None and data is None)
        or (as_type == bool and type(data) == bool)
        or (as_type == int and type(data) == int)
        or (as_type == float and type(data) == float)
        or (as_type == str and type(data) == str)
        or (as_type == bytes and type(data) == bytes)
    ):
        return data
    else:
        raise SerializeError()
