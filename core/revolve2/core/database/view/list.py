from __future__ import annotations

from functools import partial
from typing import Generic, Iterator, Type, TypeVar

from ..database import Database
from ..path import Path

T = TypeVar("T")


class List(Generic[T]):
    _database: Database
    _path: Path
    _type: Type[T]

    def __init__(self, database: Database, path: Path, type: Type[T]):
        self._database = database
        self._path = path
        self._type = type

    @classmethod
    def typed(cls, type: Type[T]) -> partial[List[T]]:
        return partial(cls, type=type)

    def __getitem__(self, index: int) -> T:
        return self._type(self._database, self._database.list_index(self._path, index))

    def append(self) -> T:
        return self._type(self._database, self._database.append_list(self._path))

    def __len__(self) -> int:
        return self._database.list_length(self._path)

    def __iter__(self) -> Iterator[T]:
        for i in range(len(self)):
            yield self._type(self._database, self._database.list_index(self._path, i))
