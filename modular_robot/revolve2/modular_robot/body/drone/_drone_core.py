from pyrr import Vector3

from ..base import DroneCore


class DroneCoreImpl(DroneCore):
    """The core module."""

    def __init__(self) -> None:
        """Initialize this object."""
        super().__init__(
            bounding_box=Vector3([0.08, 0.08, 0.04]),
            mass=0.05,
        )
