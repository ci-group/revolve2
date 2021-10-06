from typing import Optional

from .module import Module
from .serialized import Serialized
from .slot import Slot


class Brick(Module):
    front: Optional[Slot] = None
    back: Optional[Slot] = None
    left: Optional[Slot] = None
    right: Optional[Slot] = None

    def __init__(self):
        super().__init__(Module.Type.BRICK)

    def serialize(self) -> Serialized:
        """
        Serialize to a dictionary containing only the data types
        Dict, List, str, int, float bool,
        which in turn will only contain these data types as well.
        """

        serialized = super().serialize()

        if self.front is not None:
            serialized["front"] = self.front.serialize()
        if self.back is not None:
            serialized["back"] = self.back.serialize()
        if self.left is not None:
            serialized["left"] = self.left.serialize()
        if self.right is not None:
            serialized["right"] = self.right.serialize()

        return serialized
