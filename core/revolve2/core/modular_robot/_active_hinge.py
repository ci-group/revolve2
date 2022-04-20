from typing import Optional

from ._module import Module


class ActiveHinge(Module):
    ATTACHMENT = 0

    def __init__(self, rotation: float):
        super().__init__(1, rotation)

    @property
    def attachment(self) -> Optional[Module]:
        return self.children[self.ATTACHMENT]

    @attachment.setter
    def attachment(self, module: Module) -> None:
        self.children[self.ATTACHMENT] = module
