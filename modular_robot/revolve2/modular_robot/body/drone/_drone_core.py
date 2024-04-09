from pyrr import Vector3

from ..base import DroneCore


class DroneCoreImpl(DroneCore):
    """The core module of a modular robot."""

    def __init__(self):
        """
        Initialize this object.

        :param rotation: The modules' rotation.
        """
        super().__init__(
            bounding_box=Vector3([0.08, 0.08, 0.04]),
            mass=0.05,
        )
