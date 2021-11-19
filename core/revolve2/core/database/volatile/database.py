import logging
from dataclasses import dataclass
from typing import cast

from ..path import Path as PathBase
from .db_item import DbItem
from .path import Path


class Database:
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
        assert (
            type(castpath.item.item) == list
        ), "Trying to append to database item that is not a list."
        castpath.item.item.append(DbItem(None, False))
        return Path(castpath.item.item[-1])

    def insert_dict(self, path: PathBase, key: str) -> PathBase:
        castpath = self._cast_path(path)
        self._assert_not_deleted(castpath.item)
        assert (
            type(castpath.item.item) == dict
        ), "Trying to insert into database item that is not a dict."
        assert (
            key not in castpath.item.item
        ), "Key already in dict. Overwriting items not yet supported."
        newitem = DbItem(None, False)
        castpath.item.item[key] = newitem
        return Path(newitem)

    @staticmethod
    def _cast_path(path: PathBase) -> Path:
        assert type(path) == Path, "Path must be Path typecorresponding to database."
        return path

    @staticmethod
    def _assert_not_deleted(item: DbItem):
        assert (
            not item.deleted
        ), "Part of path has been deleted, invalidating this path."

    @classmethod
    def _delete_item(cls, item: DbItem):
        if type(item.item) == list:
            for child in item.item:
                cls._delete_item(child)

        if type(item.item) == dict:
            for child in item.item.values():
                cls._delete_item(child)

        item.deleted = True
