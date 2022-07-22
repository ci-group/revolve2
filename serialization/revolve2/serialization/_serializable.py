from __future__ import annotations

from abc import ABC, abstractmethod

from ._static_data import StaticData


class Serializable(ABC):
    """Interface for classes that can be serialized and deserialized to `Serializable`."""

    @abstractmethod
    def serialize(self) -> StaticData:
        """
        Serialize this object.

        :returns: The serialized object
        """
        pass

    @classmethod
    @abstractmethod
    def deserialize(cls, data: StaticData) -> Serializable:
        """
        Deserialize an instance of this class from `StaticData`.

        :param data: The data to deserialize from.
        :returns: The deserialized instance.
        """
        # This should return an instance of the class implementing this interface
        pass
