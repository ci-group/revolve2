from abc import ABC, abstractmethod

from .static_data import StaticData
from .node import Node


class Database(ABC):
    @abstractmethod
    def root(self) -> Node:
        pass
