from ..base._body import Body
from ._core_v2 import CoreV2


class BodyV2(Body):
    """Body of a V1 modular robot."""

    def __init__(self, attachment_positions: list[int] = [0] * 4) -> None:
        """
        Initialize the Body.

        :param attachment_positions: The attachment positions for child modules on Core.
        """
        self.core = CoreV2(0.0, attachment_positions)
        super().__init__()
