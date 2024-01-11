from revolve2.modular_robot.sensor_state import ActiveHingeSensorState


class ActiveHingeSensorStateImpl(ActiveHingeSensorState):
    """ActiveHingeSensorState implementation for physical robots."""

    _position: float

    def __init__(self, position: float) -> None:
        """
        Initialize this object.

        :param position: The position of the active hinge.
        """
        self._position = position

    @property
    def position(self) -> float:
        """
        Get the measured position of the active hinge.

        :returns: The measured position.
        """
        return self._position
