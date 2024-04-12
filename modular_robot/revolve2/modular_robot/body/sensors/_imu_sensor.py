from pyrr import Vector3

from ._sensor import Sensor


class IMUSensor(Sensor):
    """
    An inertial measurement unit.

    Reports specific force(closely related to acceleration), angular rate(closely related to angular velocity), and orientation.
    """

    def __init__(self, position: Vector3, rotation: float = 0.0) -> None:
        """
        Initialize the IMU sensor.

        :param rotation: The rotation of the IMU.
        :param position: The position of the IMU.
        """
        super().__init__(rotation, position)
