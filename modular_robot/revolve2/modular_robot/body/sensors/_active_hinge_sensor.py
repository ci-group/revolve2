from pyrr import Vector3

from ._sensor import Sensor


class ActiveHingeSensor(Sensor):
    """A sensors for an active hinge that measures its angle."""

    def __init__(self, rotation: float = 0.0) -> None:
        """
        Initialize the ActiveHinge sensor.

        :param rotation: The rotation of the IMU.
        """
        super().__init__(rotation, Vector3([0, 0, 0]))
