from typing import Optional

from .module import Module
from .serialized import Serialized
from .slot import Slot


class ActiveHinge(Module):
    attachment: Optional[Slot] = None

    def __init__(self):
        super().__init__(Module.Type.ACTIVE_HINGE)

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
