from pyrr import Vector3

from ..base import Core


class CoreV1(Core):
    """The core module of a modular robot."""

    def __init__(self, rotation: float):
        """
        Initialize this object.

        :param rotation: The modules' rotation.
        """
        super().__init__(
            rotation=rotation,
            bounding_box=Vector3([0.089, 0.089, 0.0603]),
            mass=0.250,
            child_offset=0.089 / 2.0,
        )
