from typing import Generic, TypeVar

from .element import Element
from .path import Path

T = TypeVar("T", bound=Element)


class List(Generic[T], Element):
    def flush(self) -> None:
        raise NotImplementedError()

    def path_is_this(path: Path) -> bool:
        raise NotImplementedError()

    def __len__(self) -> int:
        raise NotImplementedError()

    def append(self, element: Element) -> None:
        raise NotImplementedError()
