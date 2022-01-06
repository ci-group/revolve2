from typing import Optional

from .module import Module


class ActiveHinge(Module):
    ATTACHMENT_INDEX = 0

    def __init__(self, rotation: float):
        super().__init__(Module.Type.ACTIVE_HINGE, 1, rotation)

    @property
    def attachment(self) -> Optional[Module]:
        return self.children[self.ATTACHMENT_INDEX]

    @attachment.setter
    def attachment(self, module: Module) -> None:
        self.children[self.ATTACHMENT_INDEX] = module
