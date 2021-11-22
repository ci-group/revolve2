from ..database import Database
from ..path import Path


class Int:
    _database: Database
    _path: Path

    def __init__(self, database: Database, path: Path):
        self._database = database
        self._path = path

    @property
    def val(self) -> int:
        return self._database.get_int(self._path)

    @val.setter
    def val(self, val: int) -> None:
        self._database.set_int(self._path, val)
