from pyrr import Vector3, Quaternion

from ._sensor import Sensor


class ActiveHingeSensor(Sensor):
    """A sensors for an active hinge that measures its angle."""

    def __init__(self) -> None:
        """Initialize the ActiveHinge sensor."""
        super().__init__(Quaternion(), Vector3())
