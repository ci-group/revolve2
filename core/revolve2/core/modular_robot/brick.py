from typing import Optional

from .module import Module
from .serialized import Serialized
from .slot import Slot


class Brick(Module):
    FRONT = 0
    RIGHT = 1
    BACK = 2
    LEFT = 3

    def __init__(self):
        super().__init__(Module.Type.BRICK, 4)

    def serialize(self) -> Serialized:
        """
        Serialize to a dictionary containing only the data types
        Dict, List, str, int, float bool,
        which in turn will only contain these data types as well.
        """

        serialized = super().serialize()
        assert type(serialized) == dict

        if self.front is not None:
            serialized["front"] = self.front.serialize()
        if self.back is not None:
            serialized["back"] = self.back.serialize()
        if self.left is not None:
            serialized["left"] = self.left.serialize()
        if self.right is not None:
            serialized["right"] = self.right.serialize()

        return serialized

    @property
    def front(self) -> Optional[Slot]:
        return self.get_child(self.FRONT)

    @front.setter
    def front(self, slot: Slot) -> None:
        self.set_child(self.FRONT, slot)

    @property
    def right(self) -> Optional[Slot]:
        return self.get_child(self.RIGHT)

    @right.setter
    def right(self, slot: Slot) -> None:
        self.set_child(self.RIGHT, slot)

    @property
    def back(self) -> Optional[Slot]:
        return self.get_child(self.BACK)

    @back.setter
    def back(self, slot: Slot) -> None:
        self.set_child(self.BACK, slot)

    @property
    def left(self) -> Optional[Slot]:
        return self.get_child(self.LEFT)

    @left.setter
    def left(self, slot: Slot) -> None:
        self.set_child(self.LEFT, slot)
