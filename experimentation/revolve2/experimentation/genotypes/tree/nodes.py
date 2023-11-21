from __future__ import annotations

from abc import ABC, abstractmethod

from typing import Iterable, Optional, List, Self, Tuple, Union

from revolve2.modular_robot import (
    Directions,
    Module,
    Core,
    Brick,
    ActiveHinge,
    RightAngles,
)


class Node(ABC):
    def __init__(self, children: Iterable[Tuple[Directions, Optional[Node]]]) -> None:
        self._children: List[Optional[Node]] = [None] * len(self.valid_attatchments())
        for direction, child in children:
            if child is not None:
                self.set_child(child, direction)

    def get_child(self, which: Directions) -> Optional[Node]:
        if not which in self.valid_attatchments():
            raise ValueError(f"{which} is no direction of {self.__class__}")
        return self._children[which]

    def set_child(self, item: Node, which: Directions) -> None:
        if not which in self.valid_attatchments():
            raise ValueError(f"{which} is no direction of {self.__class__}")
        self._children[which] = item

    @property
    def children(self) -> List[Tuple[Directions, Node]]:
        return [
            (Directions(direction), child)
            for direction, child in enumerate(self._children)
            if child is not None
        ]

    def to_module(self) -> Module:
        ret = self._association(RightAngles.RAD_0)
        for i, child in enumerate(self._children):
            if child is not None:
                ret.set_child(child.to_module(), Directions(i))
        return ret

    def copy(self) -> Self:
        return self.__class__((d, c.copy()) for d, c in self.children)

    @property
    @abstractmethod
    def _association(cls) -> type[Modules_t]:
        ...

    @classmethod
    @abstractmethod
    def valid_attatchments(cls) -> List[Directions]:
        ...


class CoreNode(Node):
    @property
    def _association(self) -> type[Core]:
        return Core

    @classmethod
    def valid_attatchments(cls) -> List[Directions]:
        return [Directions.FRONT, Directions.LEFT, Directions.RIGHT, Directions.LEFT]


class BrickNode(Node):
    @property
    def _association(self) -> type[Brick]:
        return Brick

    @classmethod
    def valid_attatchments(cls) -> List[Directions]:
        return [Directions.FRONT, Directions.LEFT, Directions.RIGHT]


class ActiveHingeNode(Node):
    @property
    def _association(self) -> type[ActiveHinge]:
        return ActiveHinge

    @classmethod
    def valid_attatchments(cls) -> List[Directions]:
        return [Directions.FRONT]


Modules_t = Union[Core, Brick, ActiveHinge]
Nodes_t = Union[CoreNode, BrickNode, ActiveHingeNode]
