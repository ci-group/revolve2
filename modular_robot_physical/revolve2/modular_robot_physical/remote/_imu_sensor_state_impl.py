from pyrr import Vector3
from revolve2.modular_robot.sensor_state import IMUSensorState


class IMUSensorStateImpl(IMUSensorState):
    """The state of an IMU sensor."""

    _specific_force: Vector3
    _angular_rate: Vector3
    _orientation: Vector3

    def __init__(
        self, specific_force: Vector3, angular_rate: Vector3, orientation: Vector3
    ) -> None:
        """
        Initialize this object.

        :param specific_force: Speicfic force.
        :param angular_rate: Angular rate.
        :param orientation: Orientation.
        """
        self._specific_force = specific_force
        self._angular_rate = angular_rate
        self._orientation = orientation

    @property
    def specific_force(self) -> Vector3:
        """
        Get the measured specific force.

        :returns: The measured specific force.
        """
        return self._specific_force

    @property
    def angular_rate(self) -> Vector3:
        """
        Get the measured angular rate.

        :returns: The measured angular rate.
        """
        return self._angular_rate

    @property
    def orientation(self) -> Vector3:
        """
        Get the measured orientation.

        :returns: The measured orientation.
        """
        return self._orientation
