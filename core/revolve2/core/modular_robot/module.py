from __future__ import annotations

from enum import Enum
from typing import TYPE_CHECKING, List, Optional

from .serialized import Serialized

# solve cyclic dependency by only importing when type type checking
# and forward declaring
if TYPE_CHECKING:
    from .slot import Slot


class Module:
    class Type(Enum):
        CORE = "core"
        BRICK = "brick"
        ACTIVE_HINGE = "active_hinge"

    type: Type
    _children: List[Optional["Slot"]]

    def __init__(self, type: Type, num_children: int):
        self.type = type
        self._id = None
        self._children = [None] * num_children

    def serialize(self) -> Serialized:
        """
        Serialize to a dictionary containing only the data types
        Dict, List, str, int, float bool,
        which in turn will only contain these data types as well.
        """

        return {"type": self.type.value}

    @property
    def num_children(self) -> int:
        return len(self._children)

    def get_child(self, index: int) -> Optional["Slot"]:
        return self._children[index]

    def set_child(self, index: int, slot: "Slot") -> None:
        self._children[index] = slot
