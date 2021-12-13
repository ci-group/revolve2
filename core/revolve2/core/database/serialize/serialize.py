from typing import Type, TypeVar, Union

from ..data import Data, is_data
from .serializable import Serializable
from .serialize_error import SerializeError


def serialize(to_serialize: Union[Serializable, Data]) -> Data:
    if isinstance(to_serialize, Serializable):
        return to_serialize.serialize()
    elif is_data(to_serialize):
        return to_serialize
    else:
        raise SerializeError()


def deserialize(data: Data, type: Type) -> Union[Serializable, Data]:
    if issubclass(type, Serializable):
        return type.deserialize(data)
    elif is_data(data):
        return data
    else:
        raise SerializeError()
