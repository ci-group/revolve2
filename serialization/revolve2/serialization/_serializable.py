from __future__ import annotations

from abc import ABC, abstractmethod

from ._static_data import StaticData


class Serializable(ABC):
    """Interface for classes that can be serialized and deserialized to `Serializable`."""

    @abstractmethod
    def serialize(self) -> StaticData:
        """
        Serialize this object.

        :returns: The serialized object.
        """

    @classmethod
    @abstractmethod
    def deserialize(cls, data: StaticData) -> Serializable:
        """
        Deserialize an instance of this class from `StaticData`.

        This must return an instance of the class implementing this interface.

        :param data: The data to deserialize from.
        :returns: The deserialized instance.
        """
