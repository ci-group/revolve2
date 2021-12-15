from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Union

from .data import Data


class Node(ABC):
    @property
    @abstractmethod
    def data(self) -> Data:
        pass

    @data.setter
    @abstractmethod
    def data(self, data: Data) -> None:
        pass

    def __getitem__(self, key: Union[int, str]) -> Node:
        if type(key) == int:
            return self._get_list(key)
        else:
            return self._get_dict(key)

    def __setitem__(self, key: Union[int, str], data: Data) -> None:
        if type(key) == int:
            return self._set_list(key, data)
        else:
            return self._set_dict(key, data)

    @abstractmethod
    def __len__(self) -> int:
        """
        Get the length of a list.
        """
        pass

    @abstractmethod
    def append(self, data: Data) -> None:
        """
        Append to list.
        """
        pass

    @abstractmethod
    def extend(self, list: List[Data]) -> None:
        """
        Extend a list with another list. All elements appended at the end of the original list.
        """
        pass

    @abstractmethod
    def _get_list(self, index: int) -> Node:
        pass

    @abstractmethod
    def _get_dict(self, index: str) -> Node:
        pass

    @abstractmethod
    def _set_list(self, index: int, data: Data) -> None:
        pass

    @abstractmethod
    def _set_dict(self, index: str, data: Data) -> None:
        pass
