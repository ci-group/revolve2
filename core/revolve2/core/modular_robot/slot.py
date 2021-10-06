from .module import Module
from .serialized import Serialized


class Slot:
    module: Module
    rotation: float

    def __init__(self, module: Module, rotation: float):
        self.module = module
        self.rotation = rotation

    def serialize(self) -> Serialized:
        """
        Serialize to a dictionary containing only the data types
        Dict, List, str, int, float bool,
        which in turn will only contain these data types as well.
        """

        return {"module": self.module.serialize(), "rotation": self.rotation}
