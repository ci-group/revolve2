from __future__ import annotations

from ..database import Database
from ..path import Path
from .any_view import AnyView


class DictView:
    _database: Database
    _path: Path

    def __init__(self, database: Database, path: Path):
        self._database = database
        self._path = path

    def __getitem__(self, index: str) -> AnyView:
        return AnyView(self._database, self._database.dict_index(self._path, index))

    def __contains__(self, key: str) -> bool:
        return self._database.dict_has_key(self._path, key)

    def clear(self) -> None:
        self._database.set_dict(self._path)

    def insert(self, key: str) -> AnyView:
        return AnyView(self._database, self._database.insert_dict(self._path, key))
