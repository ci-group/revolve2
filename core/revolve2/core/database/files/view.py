import os
import shutil
from os import path
from typing import Dict, List, cast

from ..data import Data
from ..database_error import DatabaseError
from ..node import Node as ViewBase


class View(ViewBase):
    _path: str

    def __init__(self, path: str) -> None:
        self._path = path

    @property
    def data(self) -> Data:
        meta = self._read_meta()
        try:
            if meta[0] == "none":
                return None
            if meta[0] == "int":
                return int(self._get_value_string())
            elif meta[0] == "float":
                return float(self._get_value_string())
            elif meta[0] == "string":
                return self._get_value_string()
            elif meta[0] == "bytes":
                return self._get_value_bytes()
            elif meta[0] == "list":
                return [self._get_list(i).data for i in range(len(self))]
            elif meta[0] == "dict":
                test = {
                    dir: View(os.path.join(self._path, dir)).data
                    for dir in os.listdir(self._path)
                    if os.path.isdir(os.path.join(self._path, dir))
                }
                return test
            else:
                raise DatabaseError()
        except ValueError:
            raise DatabaseError()

    @data.setter
    def data(self, data: Data) -> None:
        self._clear_directory()
        if data is None:
            self._set_meta("none")
        elif type(data) == int:
            self._set_meta("int")
            self._set_value_string(f"{data}")
        elif type(data) == float:
            self._set_meta("float")
            self._set_value_string(f"{data}")
        elif type(data) == str:
            self._set_meta("string")
            self._set_value_string(data)
        elif type(data) == bytes:
            self._set_meta("bytes")
            self._set_value_bytes(data)
        elif type(data) == list:
            data_list = cast(List[Data], data)
            self._set_meta(f"list\n0")
            for child_data in data_list:
                self.append(child_data)
        elif type(data) == dict:
            data_dict = cast(Dict[str, Data], data)
            self._set_meta("dict")
            for key, child_data in data_dict.items():
                self[key] = child_data
        else:
            asda = type(data)
            raise DatabaseError()

    def __len__(self) -> int:
        meta = self._read_meta()
        if meta[0] == "list":
            return int(meta[1])
        else:
            raise DatabaseError()

    def append(self, data: Data) -> None:
        meta = self._read_meta()
        if meta[0] == "list":
            length = int(meta[1])
            valuedir = os.path.join(self._path, f"{length}")
            os.mkdir(valuedir)
            self._set_meta(f"list\n{length+1}")
            View(valuedir).data = data
        else:
            raise DatabaseError()

    def extend(self, list: List[Data]) -> None:
        for item in list:
            self.append(item)

    def _get_list(self, key: int) -> ViewBase:
        if key < 0:
            raise DatabaseError()

        return View(path.join(self._path, str(key)))

    def _get_dict(self, key: str) -> ViewBase:
        return View(path.join(self._path, key))

    def _set_list(self, key: int, data: Data) -> None:
        raise NotImplementedError()

    def _set_dict(self, key: str, data: Data) -> None:
        meta = self._read_meta()
        if meta[0] != "dict":
            raise IndexError("Inserting into dict, but type does not match.")
        valuedir = os.path.join(self._path, key)
        if os.path.isdir(valuedir):
            shutil.rmtree(valuedir)
        os.mkdir(valuedir)
        View(valuedir).data = data

    def _clear_directory(self) -> None:
        for root, dirs, files in os.walk(self._path):
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
