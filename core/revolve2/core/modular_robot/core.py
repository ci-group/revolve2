from typing import Optional

from .module import Module
from .serialized import Serialized


class Core(Module):
    FRONT = 0
    RIGHT = 1
    BACK = 2
    LEFT = 3

    def __init__(self, rotation: float):
        super().__init__(Module.Type.CORE, 4, rotation)

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
    def front(self) -> Optional[Module]:
        return self.children[self.FRONT]

    @front.setter
    def front(self, module: Module) -> None:
        self.children[self.FRONT] = module

    @property
    def right(self) -> Optional[Module]:
        return self.children[self.RIGHT]

    @right.setter
    def right(self, module: Module) -> None:
        self.children[self.RIGHT] = module

    @property
    def back(self) -> Optional[Module]:
        return self.children[self.BACK]

    @back.setter
    def back(self, module: Module) -> None:
        self.children[self.BACK] = module

    @property
    def left(self) -> Optional[Module]:
        return self.children[self.LEFT]

    @left.setter
    def left(self, module: Module) -> None:
        self.children[self.LEFT] = module
