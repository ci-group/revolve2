from abc import ABC, abstractmethod, abstractstaticmethod

from .path import Path


class Element(ABC):
    @abstractmethod
    def flush(self) -> None:
        pass

    @abstractstaticmethod
    def path_is_this(path: Path) -> bool:
        pass
