from ..base import Body
from ._core_v2 import CoreV2


class BodyV2(Body):
    """Body of a V2 modular robot."""

    _core: CoreV2

    def __init__(self) -> None:
        """Initialize the Body."""
        super().__init__(CoreV2(0.0))

    @property
    def core_v2(self) -> CoreV2:
        """
        Get the specific v2 core of the body.

        This function is usd since the base core has fewer attributes than a V2 core.
        Using this, allows us to specify the return type without overwriting functions of the base module.

        :return: The v2 core.
        """
        return self._core
