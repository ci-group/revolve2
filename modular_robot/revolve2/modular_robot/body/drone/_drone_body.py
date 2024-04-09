from ..base._drone_body import DroneBody
from ._drone_core import DroneCoreImpl


class DroneBodyImpl(DroneBody):
    """Body of a V1 modular robot."""

    _core: DroneCoreImpl

    def __init__(self) -> None:
        """Initialize this object."""
        super().__init__(DroneCoreImpl())

    @property
    def core_v1(self) -> DroneCoreImpl:
        """
        Get the specific v1 core of the body.

        :return: The v1 core.
        """
        return self._core
