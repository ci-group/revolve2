from .core import Core
from .serialized import Serialized


class Body:
    core: Core

    def __init__(self):
        self.core = Core()

    def serialize(self) -> Serialized:
        """
        Serialize to a dictionary containing only the data types
        Dict, List, str, int, float bool,
        which in turn will only contain these data types as well.
        """

        return {"core": self.core.serialize()}
