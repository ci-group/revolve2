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

    @abstractmethod
    def is_none(self, path: Path) -> bool:
        pass

    @abstractmethod
    def is_int(self, path: Path) -> bool:
        pass

    @abstractmethod
    def is_float(self, path: Path) -> bool:
        pass

    @abstractmethod
    def is_string(self, path: Path) -> bool:
        pass

    @abstractmethod
    def is_bytes(self, path: Path) -> bool:
        pass

    @abstractmethod
    def is_list(self, path: Path) -> bool:
        pass

    @abstractmethod
    def is_dict(self, path: Path) -> bool:
        pass

    @abstractmethod
    def get_int(self, path: Path) -> int:
        pass

    @abstractmethod
    def get_float(self, path: Path) -> float:
        pass

    @abstractmethod
    def get_string(self, path: Path) -> str:
        pass

    @abstractmethod
    def get_bytes(self, path: Path) -> bytes:
        pass

    @abstractmethod
    def list_length(self, path: Path) -> int:
        pass

    @abstractmethod
    def list_index(self, path: Path, index: int) -> Path:
        pass

    @abstractmethod
    def dict_has_key(self, path: Path, index: str) -> bool:
        pass

    @abstractmethod
    def dict_index(self, path: Path, index: str) -> Path:
        pass

    # TODO will be extended
