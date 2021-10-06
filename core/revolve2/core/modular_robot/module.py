from __future__ import annotations

from enum import Enum

from .serialized import Serialized


class Module:
    class Type(Enum):
        CORE = "core"
        BRICK = "brick"
        ACTIVE_HINGE = "active_hinge"

    type: Type

    def __init__(self, type: Type):
        self.type = type

    def serialize(self) -> Serialized:
        """
        Serialize to a dictionary containing only the data types
        Dict, List, str, int, float bool,
        which in turn will only contain these data types as well.
        """

        return {"type": str(self.type)}
