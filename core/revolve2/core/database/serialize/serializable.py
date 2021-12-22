from __future__ import annotations

from abc import ABC, abstractclassmethod, abstractmethod

from ..static_data import StaticData


class Serializable(ABC):
    @abstractmethod
    def serialize(self) -> StaticData:
        pass

    @abstractclassmethod
    def deserialize(cls, data: StaticData) -> Serializable:
        # This should return an instance of the class implementing this interface
        pass
