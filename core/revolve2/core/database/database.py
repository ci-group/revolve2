from abc import ABC, abstractmethod

from .data import Data
from .view import View


class Database(ABC):
    @abstractmethod
    def root(self) -> View:
        pass

    @abstractmethod
    def begin_transaction(self):
        pass

    @abstractmethod
    def commit_transaction(self):
        pass
