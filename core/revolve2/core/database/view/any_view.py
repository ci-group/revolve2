from typing import TYPE_CHECKING

from ..database import Database
from ..path import Path

if TYPE_CHECKING:
    from .dict_view import DictView
    from .list_view import ListView


class AnyView:
    _database: Database
    _path: Path

    def __init__(self, database: Database, path: Path):
        self._database = database
        self._path = path

    @property
    def path(self) -> Path:
        return self._path

    @property
    def int(self) -> int:
        return self._database.get_int(self._path)

    @int.setter
    def int(self, value: int) -> None:
        self._database.set_int(self._path, value)

    @property
    def float(self) -> float:
        return self._database.get_float(self._path)

    @float.setter
    def float(self, value: float) -> None:
        self._database.set_float(self._path, value)

    @property
    def string(self) -> str:
        return self._database.get_string(self._path)

    @string.setter
    def string(self, value: str) -> None:
        self._database.set_string(self._path, value)

    @property
    def bytes(self) -> bytes:
        return self._database.get_bytes(self._path)

    @bytes.setter
    def bytes(self, value: bytes) -> None:
        self._database.set_bytes(self._path, value)

    @property
    def list(self) -> "ListView":
        from .list_view import ListView

        return ListView(self._database, self._path)

    @property
    def dict(self) -> "DictView":
        from .dict_view import DictView

        return DictView(self._database, self._path)

    def is_none(self) -> bool:
        return self._database.is_none(self._path)

    def is_int(self) -> bool:
        return self._database.is_int(self._path)

    def is_bytes(self) -> bool:
        return self._database.is_bytes(self._path)

    def is_list(self) -> bool:
        return self._database.is_list(self._path)

    def is_dict(self) -> bool:
        return self._database.is_dict(self._path)

    def make_none(self) -> None:
        self._database.set_none(self._path)
