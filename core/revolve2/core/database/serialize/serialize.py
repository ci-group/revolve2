from typing import Any, Type, TypeVar, Union, cast

from ..static_data import StaticData, is_static_data
from .serializable import Serializable
from .serialize_error import SerializeError


def serialize(to_serialize: Union[Serializable, StaticData]) -> StaticData:
    if isinstance(to_serialize, Serializable):
        return to_serialize.serialize()
    elif is_static_data(to_serialize):
        return to_serialize
    else:
        raise SerializeError()


T = TypeVar("T", Serializable, StaticData)


def deserialize(data: StaticData, type: Type[T]) -> T:
    if issubclass(type, Serializable):
        return type.deserialize(data)
    elif type == StaticData and is_static_data(data):
        return data
    else:
        raise SerializeError()
