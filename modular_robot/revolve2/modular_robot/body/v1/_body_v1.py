from ..base._body import Body
from ._core_v1 import CoreV1


class BodyV1(Body):
    """Body of a V1 modular robot."""

    def __init__(self) -> None:
        """Initialize this object."""
        self.core = CoreV1(0.0)
        super().__init__()
