from ..database import Database
from ..path import Path


class BytesView:
    _database: Database
    _path: Path

    def __init__(self, database: Database, path: Path):
        self._database = database
        self._path = path

    @property
    def val(self) -> bytes:
        return self._database.get_bytes(self._path)

    @val.setter
    def val(self, val: bytes) -> None:
        self._database.set_bytes(self._path, val)
