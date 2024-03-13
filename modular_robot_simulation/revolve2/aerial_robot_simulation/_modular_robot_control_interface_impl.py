import numpy as np

from revolve2.aerial_robot import AerialRobotControlInterface
from revolve2.aerial_robot.body.base import Motor
from revolve2.simulation.scene import ControlInterface, UUIDKey

from ._build_multi_body_systems import BodyToMultiBodySystemMapping


class ModularRobotControlInterfaceImpl(AerialRobotControlInterface):
    """Implementation for ModularRobotControlInterface."""

    _simulation_control: ControlInterface
    _body_to_multi_body_system_mapping: BodyToMultiBodySystemMapping

    def __init__(
        self,
        simulation_control: ControlInterface,
        body_to_multi_body_system_mapping: BodyToMultiBodySystemMapping,
    ) -> None:
        """
        Initialize this object.

        :param simulation_control: Control interface of the actual simulation.
        :param body_to_multi_body_system_mapping: A mapping from body to multi-body system
        """
        self._simulation_control = simulation_control
        self._body_to_multi_body_system_mapping = body_to_multi_body_system_mapping

    def set_motor_target(self, motor: Motor, target: float) -> None:
        """
        Set the position target for a motor.

        :param motor: The motor to set the target for.
        :param target: The target to set.
        """
        self._simulation_control.set_motor_position_target(
            self._body_to_multi_body_system_mapping.motor_to_sim_motor[
                UUIDKey(motor)
            ],
            np.clip(target, a_min=-motor.range, a_max=motor.range),
        )
