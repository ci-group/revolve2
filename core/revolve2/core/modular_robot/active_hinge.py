from typing import Optional

from .module import Module
from .serialized import Serialized
from .slot import Slot


class ActiveHinge(Module):
    ATTACHMENT_INDEX = 0

    def __init__(self):
        super().__init__(Module.Type.ACTIVE_HINGE, 1)

    def serialize(self) -> Serialized:
        """
        Serialize to a dictionary containing only the data types
        Dict, List, str, int, float bool,
        which in turn will only contain these data types as well.
        """

        serialized = super().serialize()

        if self.attachment is not None:
            serialized["attachment"] = self.attachment.serialize()

        return serialized

    @property
    def attachment(self) -> Optional[Slot]:
        return self.get_child(self.ATTACHMENT_INDEX)

    @attachment.setter
    def attachment(self, slot: Slot) -> None:
        self.set_child(self.ATTACHMENT_INDEX, slot)
