import os
import shutil
from os import path
from typing import List, cast

from ..data import Data
from ..database_error import DatabaseError
from ..view import View as ViewBase


class View(ViewBase):
    _path: str

    def __init__(self, path: str) -> None:
        self._path = path

    @property
    def data(self) -> Data:
        meta = self._read_meta()
        try:
            if meta[0] == "int":
                return int(self._get_value_string())
            elif meta[0] == "float":
                return float(self._get_value_string())
            elif meta[0] == "string":
                return self._get_value_string()
            elif meta[0] == "bytes":
                return self._get_value_bytes()
            else:
                raise DatabaseError()
        except ValueError:
            raise DatabaseError()

    @data.setter
    def data(self, data: Data) -> None:
        self._clear_directory(self._path)
        if type(data) == "int":
            self._set_meta("int")
            self._set_value_string(f"{data}")
        elif type(data) == "float":
            self._set_meta("float")
            self._set_value_string(f"{data}")
        elif type(data) == "str":
            self._set_meta("string")
            self._set_value_string(data)
        elif type(data) == "bytes":
            self._set_meta("bytes")
            self._set_value_bytes(data)
        elif type(data) == list:
            data_list = cast(List[Data], data)
            self._set_meta(f"list\n{len(data_list)}")
            for child_data in data_list:
                self.append(child_data)
        else:
            raise DatabaseError()

    def __len__(self) -> int:
        raise NotImplementedError()

    def append(self, data: Data) -> None:
        raise NotImplementedError()

    def extend(self, list: List[Data]) -> None:
        raise NotImplementedError()

    def _get_list(self, key: int) -> ViewBase:
        return View(path.join(self._path, str(key)))

    def _get_dict(self, key: str) -> ViewBase:
        return View(path.join(self._path, key))

    def _set_list(self, key: int, data: Data) -> None:
        raise NotImplementedError()

    def _set_dict(self, key: str, data: Data) -> None:
        raise NotImplementedError()

    def _clear_directory(self, path: str) -> None:
        for root, dirs, files in os.walk(path):
            for f in files:
                os.unlink(os.path.join(root, f))
            for d in dirs:
                shutil.rmtree(os.path.join(root, d))

    def _meta_path(self) -> str:
        return os.path.join(self._path, ".meta")

    def _value_path(self) -> str:
        return os.path.join(self._path, "value")

    def _read_meta(self) -> List[str]:
        try:
            with open(self._meta_path(), "r") as f:
                meta = f.read().splitlines()
        except FileNotFoundError:
            raise DatabaseError()
        return meta

    def _set_meta(self, value: str) -> None:
        with open(self._meta_path(), "w") as f:
            f.write(value)

    def _get_value_string(self) -> str:
        with open(self._value_path(), "r") as f:
            return f.read()

    def _get_value_bytes(self) -> bytes:
        with open(self._value_path(), "rb") as f:
            return f.read()

    def _set_value_string(self, value: str) -> None:
        with open(self._value_path(), "w") as f:
            f.write(f"{value}")

    def _set_value_bytes(self, value: bytes) -> None:
        with open(self._value_path(), "wb") as f:
            f.write(value)
