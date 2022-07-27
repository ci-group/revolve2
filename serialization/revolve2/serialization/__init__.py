"""
Serialization interfaces and tools.

Deprecation warning: this library will be removed in a nearby version of Revolve2.
"""

from ._functions import deserialize, serialize
from ._serializable import Serializable
from ._serialize_error import SerializeError
from ._static_data import StaticData, is_static_data

__all__ = [
    "Serializable",
    "SerializeError",
    "StaticData",
    "deserialize",
    "is_static_data",
    "serialize",
]
