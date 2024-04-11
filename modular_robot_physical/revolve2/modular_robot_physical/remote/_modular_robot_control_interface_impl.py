from revolve2.modular_robot import ModularRobotControlInterface
from revolve2.modular_robot.body.base import ActiveHinge, Motor

from .._uuid_key import UUIDKey


class ModularRobotControlInterfaceImpl(ModularRobotControlInterface):
    """Implementation of ModularRobotControlInterface."""

    _set_active_hinges: list[tuple[UUIDKey[ActiveHinge], float]]
    _set_motors: list[tuple[UUIDKey[Motor], float]]

    def __init__(self) -> None:
        """Initialize this object."""
        self._set_active_hinges = []

    def set_active_hinge_target(self, active_hinge: ActiveHinge, target: float) -> None:
        """
        Set the position target for an active hinge.

        Target is clamped within the active hinges range.

        :param active_hinge: The active hinge to set the target for.
        :param target: The target to set.
        """
        self._set_active_hinges.append((UUIDKey(active_hinge), target))

    def set_motor_target(self, motor: Motor, target: float) -> None:
        """
        Set the target for an motor.

        :param motor: The motor to set the target for.
        :param target: The target to set.
        """
        self._set_motors.append((UUIDKey(motor), target))