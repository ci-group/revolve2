from __future__ import annotations

from typing import Iterator

from ..database import Database
from ..path import Path
from .any_view import AnyView


class ListView:
    _database: Database
    _path: Path

    def __init__(self, database: Database, path: Path):
        self._database = database
        self._path = path

    def __getitem__(self, index: int) -> AnyView:
        return AnyView(self._database, self._database.list_index(self._path, index))

    def clear(self) -> None:
        self._database.set_list(self._path)

    def append(self) -> AnyView:
        return AnyView(self._database, self._database.append_list(self._path))

    def __len__(self) -> int:
        return self._database.list_length(self._path)

    def __iter__(self) -> Iterator[AnyView]:
        for i in range(len(self)):
            yield AnyView(self._database, self._database.list_index(self._path, i))
