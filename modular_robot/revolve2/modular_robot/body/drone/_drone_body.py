from ..base._drone_body import DroneBody
from ._drone_core import DroneCoreImpl


class DroneBodyImpl(DroneBody):
    """Body of a drone modular robot."""

    _core: DroneCoreImpl

    def __init__(self) -> None:
        """Initialize this object."""
        super().__init__(DroneCoreImpl())

    @property
    def core(self) -> DroneCoreImpl:
        """
        Get the specific core of the body.

        :return: The drone core.
        """
        return self._core
