"""Classes and interfaces for working with databases."""

from ._incompatible_error import IncompatibleError
from ._serializable import Serializable
from ._serializable_frozen_struct import SerializableFrozenStruct
from ._serializable_incrementable_struct import SerializableIncrementableStruct
from ._serializable_list import SerializableList
from ._serializable_struct import SerializableStruct
from ._serializer import Serializer
from ._sqlite import open_async_database_sqlite, open_database_sqlite

__all__ = [
    "IncompatibleError",
    "Serializable",
    "SerializableFrozenStruct",
    "SerializableIncrementableStruct",
    "SerializableList",
    "SerializableStruct",
    "Serializer",
    "open_async_database_sqlite",
    "open_database_sqlite",
]
