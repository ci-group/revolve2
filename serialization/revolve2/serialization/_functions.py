from __future__ import annotations

from typing import Type, TypeVar, Union

from ._serializable import Serializable
from ._serialize_error import SerializeError
from ._static_data import StaticData, is_static_data


def serialize(to_serialize: Union[Serializable, StaticData]) -> StaticData:
    """
    Deserialize the provided object.

    Abstracts away wether the object is a `Serializable` class or already StaticData.
    In the latter case the object is returned as is.

    :param to_serialize: The object to serialize.
    :returns: The serialized object.
    :raises SerializeError: When the object cannot be serialized.
    """
    if isinstance(to_serialize, Serializable):
        return to_serialize.serialize()
    elif is_static_data(to_serialize):
        return to_serialize
    else:
        raise SerializeError()


T = TypeVar("T", Serializable, StaticData)


def deserialize(data: StaticData, as_type: Type[T]) -> T:
    """
    Deserialize `StaticData` to the given type.

    :param data: The `StaticData` to deserialize from.
    :param as_type: The type to deserialize to.
    :returns: The deserialized object.
    :raises SerializeError: When the object cannot be serialized.
    """
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
