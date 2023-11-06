from ..base._body import Body
from ._core_v2 import CoreV2


class BodyV2(Body):
    """Body of a V1 modular robot."""

    def __init__(self) -> None:
        """Initialize the Body."""
        self._core: CoreV2 = CoreV2(0.0)
        super().__init__()

    @property
    def core(self) -> CoreV2:
        """
        Return the core.

        :return: The core.
        """
        return self._core
