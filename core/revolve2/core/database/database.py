from abc import ABC, abstractmethod

from .path import Path


class Database(ABC):
    @abstractmethod
    def root(self) -> Path:
        raise NotImplementedError()

    @abstractmethod
    def begin_transaction(self):
        pass

    @abstractmethod
    def commit_transaction(self):
        pass

    @abstractmethod
    def set_none(self, path: Path) -> None:
        pass

    @abstractmethod
    def set_int(self, path: Path, value: int) -> None:
        pass

    @abstractmethod
    def set_float(self, path: Path, value: float) -> None:
        pass

    @abstractmethod
    def set_string(self, path: Path, value: str) -> None:
        pass

    @abstractmethod
    def set_bytes(self, path: Path, value: bytes) -> None:
        pass

    @abstractmethod
    def set_list(self, path: Path) -> None:
        pass

    @abstractmethod
    def set_dict(self, path: Path) -> None:
        pass

    @abstractmethod
    def append_list(self, path: Path) -> Path:
        pass

    @abstractmethod
    def insert_dict(self, path: Path, key: str) -> Path:
        pass

    # TODO also add stuff to remove things
