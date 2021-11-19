from .database import Database
from .element import Element
from .path import Path


class IntView(Element):
    _database: Database
    _path: Path

    def __init__(self, database: Database, path: Path):
        self._database = database
        self._path = path

    def flush(self) -> None:
        # This element does not stage anything
        pass

    def path_is_this(path: Path) -> bool:
        pass
