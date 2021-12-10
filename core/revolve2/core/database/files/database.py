import logging

from ..data import Data
from ..database import Database as DatabaseBase
from ..view import View as ViewBase
from .view import View


class Database(DatabaseBase):
    _root_directory: str

    def __init__(self, root_directory: str) -> None:
        self._root_directory = root_directory

    def root(self) -> ViewBase:
        return View(self._root_directory)

    def begin_transaction(self):
        logging.warning(
            "Transaction not implemented for files database. Continuing without transaction.."
        )  # TODO

    def commit_transaction(self):
        logging.warning(
            "Transaction not implemented for files database. Commit transaction ignored."
        )  # TODO
