from __future__ import annotations

from abc import ABC, abstractmethod

from .._static_data import StaticData


class Serializable(ABC):
    @abstractmethod
    def serialize(self) -> StaticData:
        pass

    @classmethod
    @abstractmethod
    def deserialize(cls, data: StaticData) -> Serializable:
        # This should return an instance of the class implementing this interface
        pass
