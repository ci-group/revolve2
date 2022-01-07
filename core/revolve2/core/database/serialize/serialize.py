from typing import Any, Type, Union

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


def deserialize(
    data: StaticData, type: Union[Type[Serializable], Type[StaticData]]
) -> Union[Serializable, StaticData]:
    if issubclass(type, Serializable):
        return type.deserialize(data)
    elif is_static_data(data):
        return data
    else:
        raise SerializeError()
