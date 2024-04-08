from ..base._drone_body import DroneBody
from ._drone_core_v1 import DroneCoreV1


class DroneBodyV1(DroneBody):
    """Body of a V1 modular robot."""

    _core: DroneCoreV1

    def __init__(self) -> None:
        """Initialize this object."""
        super().__init__(DroneCoreV1())

    @property
    def core_v1(self) -> DroneCoreV1:
        """
        Get the specific v1 core of the body.

        :return: The v1 core.
        """
        return self._core
