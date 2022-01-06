from __future__ import annotations

from enum import Enum
from typing import TYPE_CHECKING, List, Optional

from .serialized import Serialized


class Module:
    class Type(Enum):
        CORE = "core"
        BRICK = "brick"
        ACTIVE_HINGE = "active_hinge"

    _type: Type
    _children: List[Optional[Module]]
    _rotation: float

    def __init__(self, type: Type, num_children: int, rotation: float):
        self._type = type
        self._id = None
        self._children = [None] * num_children
        self._rotation = rotation

    def serialize(self) -> Serialized:
        """
        Serialize to a dictionary containing only the data types
        Dict, List, str, int, float bool,
        which in turn will only contain these data types as well.
        """

        return {"type": self.type.value}

    @property
    def type(self) -> Type:
        return self._type

    @property
    def children(self) -> List[Optional[Module]]:
        return self._children

    @property
    def rotation(self) -> float:
        return self._rotation
