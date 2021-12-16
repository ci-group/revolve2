from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List

from .dynamic_list import DynamicList
from .static_data import StaticData


class Node(ABC):
    # @abstractmethod
    # def as_static_data(self) -> StaticData:
    #     """
    #     If this node is static data, get the data.
    #     Else, raise DatabaseError.
    #     """

    # @abstractmethod
    # def as_dynamic_list(self) -> DynamicList:
    #     """
    #     If this node is a dynamic list, get a dynamic list handle.
    #     Else, raise DatabaseError.
    #     """

    @abstractmethod
    def set_static_data(self, data: StaticData) -> None:
        """
        Make this node static data and set the
        """


class Node(ABC):
    @property
    @abstractmethod
    def data(self) -> StaticData:
        pass

    @data.setter
    @abstractmethod
    def data(self, data: StaticData) -> None:
        pass

    def __getitem__(self, key: int) -> Node:
        """
        Get item from list.
        """

    # @abstractmethod
    # def __len__(self) -> int:
    #    """
    #    Get the length of a list.
    #    """
    #    pass

    @abstractmethod
    def append(self, data: StaticData) -> None:
        """
        Append to list.
        """
        pass

    @abstractmethod
    def extend(self, list: List[StaticData]) -> None:
        """
        Extend a list with another list. All elements appended at the end of the original list.
        """
        pass

    @abstractmethod
    def begin_transaction(self):
        pass

    @abstractmethod
    def commit_transaction(self):
        pass
