from ..base._body import Body
from ._core_v1 import CoreV1


class BodyV1(Body):
    """Body of a V1 modular robot."""

    _core: CoreV1

    def __init__(self) -> None:
        """Initialize this object."""
        super().__init__(CoreV1(0.0))

    @property
    def core_v1(self) -> CoreV1:
        """
<<<<<<< HEAD
        Get the specific v1 core of the body.
=======
        Get the core of the body.
>>>>>>> 60cc945 (feedback)

        :return: The v1 core.
        """
        return self._core
