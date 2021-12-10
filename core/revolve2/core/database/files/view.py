from os import path

from ..data import Data
from ..view import View as ViewBase


class View(ViewBase):
    _path: str

    def __init__(self, path: str) -> None:
        self._path = path

    @property
    def data(self) -> Data:
        raise NotImplementedError()

    @data.setter
    def data(self, data: Data) -> None:
        raise NotImplementedError()

    def _get_list(self, key: int) -> ViewBase:
        return View(path.join(self._path, str(key)))

    def _get_dict(self, key: str) -> ViewBase:
        return View(path.join(self._path, key))

    def _set_list(self, key: int, data: Data) -> None:
        raise NotImplementedError()

    def _set_dict(self, key: str, data: Data) -> None:
        raise NotImplementedError()
