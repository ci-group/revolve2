from __future__ import annotations

from abc import ABC, abstractclassmethod, abstractmethod

from ..view import AnyView


class Serializable(ABC):
    @abstractmethod
    def to_database(self, db_view: AnyView) -> None:
        pass

    @abstractclassmethod
    def from_database(cls, db_view: AnyView) -> Serializable:
        # This should return an instance of the class implementing this interface
        pass
