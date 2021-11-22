import logging
from dataclasses import dataclass
from typing import cast

from ..database import Database as DatabaseBase
from ..path import Path as PathBase
from .db_item import DbItem
from .path import Path


class Database(DatabaseBase):
    """
    In memory implementation of database.
    Not optimized at all. Meant for debugging.
    """

    _db: DbItem

    def __init__(self):
        self._db = None

    def root(self) -> PathBase:
        self._db = DbItem(None, False)
        return Path(self._db)

    def begin_transaction(self):
        logging.warning(
            "Transaction not implemented for volatile database. Continuing without transaction.."
        )

    def commit_transaction(self):
        logging.warning(
            "Transaction not implemented for volatile database. Commit transaction ignored."
        )

    def set_none(self, path: PathBase) -> None:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)

        item = castpath.item

        if type(item.item) == list:
            for child in item.item:
                self._delete_item(child)

        if type(item.item) == dict:
            for child in item.item.values():
                self._delete_item(child)

        item.item = None

    def set_int(self, path: PathBase, value: int) -> None:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        self.set_none(castpath)
        castpath.item.item = value

    def set_float(self, path: PathBase, value: float) -> None:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        self.set_none(castpath)
        castpath.item.item = value

    def set_string(self, path: PathBase, value: str) -> None:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        self.set_none(castpath)
        castpath.item.item = value

    def set_bytes(self, path: PathBase, value: bytes) -> None:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        self.set_none(castpath)
        castpath.item.item = value

    def set_list(self, path: PathBase) -> None:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        self.set_none(castpath)
        castpath.item.item = []

    def set_dict(self, path: PathBase) -> None:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        self.set_none(castpath)
        castpath.item.item = {}

    def append_list(self, path: PathBase) -> PathBase:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        if not type(castpath.item.item) == list:
            raise RuntimeError("Trying to append to database item that is not a list.")
        castpath.item.item.append(DbItem(None, False))
        return Path(castpath.item.item[-1])

    def insert_dict(self, path: PathBase, key: str) -> PathBase:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        if not type(castpath.item.item) == dict:
            raise RuntimeError(
                "Trying to insert into database item that is not a dict."
            )
        if key in castpath.item.item:
            raise RuntimeError(
                "Key already in dict. Overwriting items not yet supported."
            )
        newitem = DbItem(None, False)
        castpath.item.item[key] = newitem
        return Path(newitem)

    def is_none(self, path: Path) -> bool:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        return path.item.item is None

    def is_int(self, path: Path) -> bool:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        return type(path.item.item) == int

    def is_float(self, path: Path) -> bool:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        return type(path.item.item) == float

    def is_string(self, path: Path) -> bool:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        return type(path.item.item) == str

    def is_bytes(self, path: Path) -> bool:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        return type(path.item.item) == bytes

    def is_list(self, path: Path) -> bool:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        return type(path.item.item) == list

    def is_dict(self, path: Path) -> bool:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        return type(path.item.item) == dict

    def get_int(self, path: Path) -> int:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        if not self.is_int(path):
            raise RuntimeError("Trying to get int, but type does not match.")
        return castpath.item.item

    def get_float(self, path: Path) -> float:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        if not self.is_float(path):
            raise RuntimeError("Trying to get float, but type does not match.")
        return castpath.item.item

    def get_string(self, path: Path) -> str:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        if not self.is_string(path):
            raise RuntimeError("Trying to get string, but type does not match.")
        return castpath.item.item

    def get_bytes(self, path: Path) -> bytes:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        if not self.is_bytes(path):
            raise RuntimeError("Trying to get bytes, but type does not match.")
        return castpath.item.item

    def list_length(self, path: Path) -> int:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        if not self.is_list(path):
            raise RuntimeError("Trying to length of list, but type does not match.")
        return len(castpath.item.item)

    def list_index(self, path: Path, index: int) -> Path:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        if not self.is_list(path):
            raise RuntimeError("Trying to index list, but type does not match.")
        if not index < len(castpath.item.item):
            raise RuntimeError("Trying to index list out of bounds.")
        return Path(castpath.item.item[index])

    def dict_has_key(self, path: Path, index: str) -> bool:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        if not self.is_dict(path):
            raise RuntimeError("Trying check if dict has key, but type does not match.")
        return index in castpath.item.item

    def dict_index(self, path: Path, index: str) -> Path:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        if not self.is_dict(path):
            raise RuntimeError("Trying to index dict, but type does not match.")
        if not self.dict_has_key(path, index):
            raise RuntimeError("Trying to index dict, but key not in dict.")
        return Path(castpath.item.item[index])

    @staticmethod
    def _cast_path(path: PathBase) -> Path:
        if not type(path) == Path:
            raise RuntimeError("Path must be Path typecorresponding to database.")
        return path

    @staticmethod
    def _assert_not_deleted(item: DbItem):
        if item.deleted:
            raise RuntimeError("Part of path has been deleted, invalidating this path.")

    @classmethod
    def _delete_item(cls, item: DbItem):
        if type(item.item) == list:
            for child in item.item:
                cls._delete_item(child)

        if type(item.item) == dict:
            for child in item.item.values():
                cls._delete_item(child)

        item.deleted = True
