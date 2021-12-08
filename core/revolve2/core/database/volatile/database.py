import logging
from typing import Dict, List, cast

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
            raise IndexError("Trying to append to database item that is not a list.")
        castpath.item.item.append(DbItem(None, False))
        return Path(castpath.item.item[-1])

    def insert_dict(self, path: PathBase, key: str) -> PathBase:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        if not type(castpath.item.item) == dict:
            raise IndexError("Trying to insert into database item that is not a dict.")
        if key in castpath.item.item:
            raise IndexError(
                "Key already in dict. Overwriting items not yet supported."
            )
        newitem = DbItem(None, False)
        castpath.item.item[key] = newitem
        return Path(newitem)

    def is_none(self, path: PathBase) -> bool:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        return castpath.item.item is None

    def is_int(self, path: PathBase) -> bool:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        return type(castpath.item.item) == int

    def is_float(self, path: PathBase) -> bool:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        return type(castpath.item.item) == float

    def is_string(self, path: PathBase) -> bool:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        return type(castpath.item.item) == str

    def is_bytes(self, path: PathBase) -> bool:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        return type(castpath.item.item) == bytes

    def is_list(self, path: PathBase) -> bool:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        return type(castpath.item.item) == list

    def is_dict(self, path: PathBase) -> bool:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        return type(castpath.item.item) == dict

    def get_int(self, path: PathBase) -> int:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        if not self.is_int(castpath):
            raise IndexError("Trying to get int, but type does not match.")
        return cast(int, castpath.item.item)

    def get_float(self, path: PathBase) -> float:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        if not self.is_float(castpath):
            raise IndexError("Trying to get float, but type does not match.")
        return cast(float, castpath.item.item)

    def get_string(self, path: PathBase) -> str:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        if not self.is_string(castpath):
            raise IndexError("Trying to get string, but type does not match.")
        return cast(str, castpath.item.item)

    def get_bytes(self, path: PathBase) -> bytes:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        if not self.is_bytes(castpath):
            raise IndexError("Trying to get bytes, but type does not match.")
        return cast(bytes, castpath.item.item)

    def list_length(self, path: PathBase) -> int:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        if not self.is_list(castpath):
            raise IndexError("Trying to length of list, but type does not match.")
        return len(cast(List[DbItem], castpath.item.item))

    def list_index(self, path: PathBase, index: int) -> Path:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        if not self.is_list(castpath):
            raise IndexError("Trying to index list, but type does not match.")
        if not index < len(cast(List[DbItem], castpath.item.item)):
            raise IndexError("Trying to index list out of bounds.")
        return Path(cast(List[DbItem], castpath.item.item)[index])

    def dict_has_key(self, path: PathBase, index: str) -> bool:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        if not self.is_dict(castpath):
            raise IndexError("Trying check if dict has key, but type does not match.")
        return index in cast(Dict[str, DbItem], castpath.item.item)

    def dict_index(self, path: PathBase, index: str) -> Path:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        if not self.is_dict(castpath):
            raise IndexError("Trying to index dict, but type does not match.")
        if not self.dict_has_key(path, index):
            raise IndexError("Trying to index dict, but key not in dict.")
        return Path(cast(Dict[str, DbItem], castpath.item.item)[index])

    @staticmethod
    def _cast_path(path: PathBase) -> Path:
        if not type(path) == Path:
            raise IndexError("Path must be Path typecorresponding to database.")
        return path

    @staticmethod
    def _assert_not_deleted(item: DbItem):
        if item.deleted:
            raise IndexError("Part of path has been deleted, invalidating this path.")

    @classmethod
    def _delete_item(cls, item: DbItem):
        if type(item.item) == list:
            for child in item.item:
                cls._delete_item(child)

        if type(item.item) == dict:
            for child in item.item.values():
                cls._delete_item(child)

        item.deleted = True
