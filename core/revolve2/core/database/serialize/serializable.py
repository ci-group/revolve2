from __future__ import annotations

from abc import ABC, abstractclassmethod, abstractmethod

from ..data import Data


class Serializable(ABC):
    @abstractmethod
    def serialize(self) -> Data:
        pass

    @abstractclassmethod
    def deserialize(cls, data: Data) -> Serializable:
        # This should return an instance of the class implementing this interface
        pass
