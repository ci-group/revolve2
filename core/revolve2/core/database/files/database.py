import logging
import os
import shutil
from typing import List, Union

from revolve2.core.database import Database as DatabaseBase

from ..path import Path as PathBase
from .path import Path


class Database(DatabaseBase):
    """
    All functions throw IndexError when something goes wrong. TODO better error choice
    """

    _root_directory: str

    def __init__(self, root_directory: str):
        self._root_directory = root_directory
        if not os.path.isdir(self._root_directory):
            os.makedirs(self._root_directory, exist_ok=True)
            self.set_none(self.root())

    def root(self) -> PathBase:
        return Path(self._root_directory)

    def begin_transaction(self):
        logging.warning(
            "Transaction not implemented for files database. Continuing without transaction.."
        )

    def commit_transaction(self):
        logging.warning(
            "Transaction not implemented for files database. Commit transaction ignored."
        )

    def set_none(self, path: PathBase) -> None:
        castpath = self._cast_path(path)
        self._clear_directory(castpath.path)
        self._set_meta(castpath, "none")

    def set_int(self, path: PathBase, value: int) -> None:
        castpath = self._cast_path(path)
        self._clear_directory(castpath.path)
        self._set_meta(castpath, "int")
        self._set_value_string(castpath, f"{value}")

    def set_float(self, path: PathBase, value: float) -> None:
        castpath = self._cast_path(path)
        self._clear_directory(castpath.path)
        self._set_meta(castpath, "float")
        self._set_value_string(castpath, f"{value}")

    def set_string(self, path: PathBase, value: str) -> None:
        castpath = self._cast_path(path)
        self._clear_directory(castpath.path)
        self._set_meta(castpath, "string")
        self._set_value_string(castpath, f"{value}")

    def set_bytes(self, path: PathBase, value: bytes) -> None:
        castpath = self._cast_path(path)
        self._clear_directory(castpath.path)
        self._set_meta(castpath, "bytes")
        self._set_value_bytes(castpath, value)

    def set_list(self, path: PathBase) -> None:
        castpath = self._cast_path(path)
        self._clear_directory(castpath.path)
        self._set_meta(castpath, "list\n0")

    def set_dict(self, path: PathBase) -> None:
        castpath = self._cast_path(path)
        self._clear_directory(castpath.path)
        self._set_meta(castpath, "dict")

    def append_list(self, path: PathBase) -> PathBase:
        castpath = self._cast_path(path)
        meta = self._read_meta(castpath)
        if meta[0] != "list":
            raise IndexError("Appending to list, but type does not match.")
        length = int(meta[1])
        valuedir = os.path.join(castpath.path, f"{length}")
        os.mkdir(valuedir)
        self._set_meta(castpath, f"list\n{length+1}")
        return Path(valuedir)

    def insert_dict(self, path: PathBase, index: str) -> PathBase:
        castpath = self._cast_path(path)
        meta = self._read_meta(castpath)
        if meta[0] != "dict":
            raise IndexError("Inserting into dict, but type does not match.")
        valuedir = os.path.join(castpath.path, index)
        if os.path.isdir(valuedir):
            shutil.rmtree(valuedir)
        os.mkdir(valuedir)
        return Path(valuedir)

    def is_none(self, path: PathBase) -> bool:
        castpath = self._cast_path(path)
        meta = self._read_meta(castpath)
        return meta[0] == "none"

    def is_int(self, path: PathBase) -> bool:
        castpath = self._cast_path(path)
        meta = self._read_meta(castpath)
        return meta[0] == "int"

    def is_float(self, path: PathBase) -> bool:
        castpath = self._cast_path(path)
        meta = self._read_meta(castpath)
        return meta[0] == "float"

    def is_string(self, path: PathBase) -> bool:
        castpath = self._cast_path(path)
        meta = self._read_meta(castpath)
        return meta[0] == "string"

    def is_bytes(self, path: PathBase) -> bool:
        castpath = self._cast_path(path)
        meta = self._read_meta(castpath)
        return meta[0] == "bytes"

    def is_list(self, path: PathBase) -> bool:
        castpath = self._cast_path(path)
        meta = self._read_meta(castpath)
        return meta[0] == "list"

    def is_dict(self, path: PathBase) -> bool:
        castpath = self._cast_path(path)
        meta = self._read_meta(castpath)
        return meta[0] == "dict"

    def get_int(self, path: PathBase) -> int:
        castpath = self._cast_path(path)
        meta = self._read_meta(castpath)
        if meta[0] != "int":
            raise IndexError("Appending to list, but type does not match.")
        return int(self._get_value_string(castpath))

    def get_float(self, path: PathBase) -> float:
        castpath = self._cast_path(path)
        meta = self._read_meta(castpath)
        if meta[0] != "float":
            raise IndexError("Appending to list, but type does not match.")
        return float(self._get_value_string(castpath))

    def get_string(self, path: PathBase) -> str:
        castpath = self._cast_path(path)
        meta = self._read_meta(castpath)
        if meta[0] != "string":
            raise IndexError("Appending to list, but type does not match.")
        return self._get_value_string(castpath)

    def get_bytes(self, path: PathBase) -> bytes:
        castpath = self._cast_path(path)
        meta = self._read_meta(castpath)
        if meta[0] != "bytes":
            raise IndexError("Appending to list, but type does not match.")
        return self._get_value_bytes(castpath)

    def list_length(self, path: PathBase) -> int:
        castpath = self._cast_path(path)
        meta = self._read_meta(castpath)
        if meta[0] != "list":
            raise IndexError("Getting length of list, buyt type does not match.")
        return int(meta[1])

    def list_index(self, path: PathBase, index: int) -> PathBase:
        castpath = self._cast_path(path)
        meta = self._read_meta(castpath)
        if meta[0] != "list":
            raise IndexError("Indexing list, but type does not match.")

        if index >= int(meta[1]):
            raise IndexError("Indexing list out of bounds.")

        return Path(os.path.join(castpath.path, f"{index}"))

    def dict_has_key(self, path: PathBase, index: str) -> bool:
        castpath = self._cast_path(path)
        meta = self._read_meta(castpath)
        if meta[0] != "dict":
            raise IndexError("Checking if dict has index, but type does not match.")

        valuedir = os.path.join(castpath.path, index)
        return os.path.isdir(valuedir)

    def dict_index(self, path: PathBase, index: str) -> PathBase:
        castpath = self._cast_path(path)
        meta = self._read_meta(castpath)
        if meta[0] != "dict":
            raise IndexError("Indexing dict, but type does not match.")

        valuedir = os.path.join(castpath.path, index)
        if not os.path.isdir(valuedir):
            raise IndexError("Dict index does not exist")
        return Path(valuedir)

    def _clear_directory(self, path: str) -> None:
        for root, dirs, files in os.walk(path):
            for f in files:
                os.unlink(os.path.join(root, f))
            for d in dirs:
                shutil.rmtree(os.path.join(root, d))

    @staticmethod
    def _cast_path(path: PathBase) -> Path:
        if not type(path) == Path:
            raise IndexError("Path must be Path typecorresponding to database.")
        return path

    @staticmethod
    def _meta_path(path: Path) -> str:
        return os.path.join(path.path, ".meta")

    @classmethod
    def _read_meta(cls, path: Path) -> List[str]:
        with open(cls._meta_path(path), "r") as f:
            meta = f.read().splitlines()
        return meta

    @classmethod
    def _set_meta(cls, path: Path, value: str) -> None:
        with open(cls._meta_path(path), "w") as f:
            f.write(value)

    @staticmethod
    def _value_path(path: Path) -> str:
        return os.path.join(path.path, "value")

    @classmethod
    def _set_value_string(cls, path: Path, value: str) -> None:
        with open(cls._value_path(path), "w") as f:
            f.write(f"{value}")

    @classmethod
    def _set_value_bytes(cls, path: Path, value: bytes) -> None:
        with open(cls._value_path(path), "wb") as f:
            f.write(value)

    @classmethod
    def _get_value_string(cls, path: Path) -> None:
        with open(cls._value_path(path), "r") as f:
            return f.read()

    @classmethod
    def _get_value_bytes(cls, path: Path) -> None:
        with open(cls._value_path(path), "rb") as f:
            return f.read()
