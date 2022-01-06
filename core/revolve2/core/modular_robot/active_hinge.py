from typing import Optional

from .module import Module
from .serialized import Serialized


class ActiveHinge(Module):
    ATTACHMENT_INDEX = 0

    def __init__(self, rotation: float):
        super().__init__(Module.Type.ACTIVE_HINGE, 1, rotation)

    def serialize(self) -> Serialized:
        """
        Serialize to a dictionary containing only the data types
        Dict, List, str, int, float bool,
        which in turn will only contain these data types as well.
        """

        serialized = super().serialize()
        assert type(serialized) == dict

        if self.attachment is not None:
            serialized["attachment"] = self.attachment.serialize()

        return serialized

    @property
    def attachment(self) -> Optional[Module]:
        return self.children[self.ATTACHMENT_INDEX]

    @attachment.setter
    def attachment(self, module: Module) -> None:
        self.children[self.ATTACHMENT_INDEX] = module
