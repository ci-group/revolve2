from typing import TYPE_CHECKING

from ..database import Database
from ..path import Path
from .bytes_view import BytesView
from .int_view import IntView

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
    def int(self) -> IntView:
        return IntView(self._database, self._path)

    @property
    def bytes(self) -> BytesView:
        return BytesView(self._database, self._path)

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
