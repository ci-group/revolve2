from typing import Generic, TypeVar

from .element import Element
from .path import Path

T = TypeVar("T", bound=Element)


class Dict(Generic[T], Element):
    def flush(self) -> None:
        raise NotImplementedError()

    def path_is_this(path: Path) -> bool:
        raise NotImplementedError()

    def __setitem__(self, key: str, value: Element):
        raise NotImplementedError()

    def __getitem__(self, key: str):
        raise NotImplementedError()
