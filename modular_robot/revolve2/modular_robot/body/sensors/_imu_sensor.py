from pyrr import Vector3, Quaternion

from ._sensor import Sensor


class IMUSensor(Sensor):
    """
    An inertial measurement unit (IMU).

    Reports specific force(closely related to acceleration), angular rate(closely related to angular velocity), and orientation.
    """

    def __init__(
        self, position: Vector3, orientation: Quaternion = Quaternion()
    ) -> None:
        """
        Initialize the IMU sensor.

        :param orientation: The rotation of the IMU.
        :param position: The position of the IMU.
        """
        super().__init__(orientation, position)
