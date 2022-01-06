from typing import Optional

from .module import Module


class Brick(Module):
    FRONT = 0
    RIGHT = 1
    LEFT = 2

    def __init__(self, rotation: float):
        super().__init__(Module.Type.BRICK, 4, rotation)

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
    def left(self) -> Optional[Module]:
        return self.children[self.LEFT]

    @left.setter
    def left(self, module: Module) -> None:
        self.children[self.LEFT] = module
