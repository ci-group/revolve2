import numpy as np

from revolve2.modular_robot import ModularRobotControlInterface
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.simulation.scene import ControlInterface, UUIDKey

from ._build_multi_body_systems import BodyToMultiBodySystemMapping


class ModularRobotControlInterfaceImpl(ModularRobotControlInterface):
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

    def set_active_hinge_target(self, active_hinge: ActiveHinge, target: float) -> None:
        """
        Set the position target for an active hinge.

        :param active_hinge: The active hinge to set the target for.
        :param target: The target to set.
        """
        self._simulation_control.set_joint_hinge_position_target(
            self._body_to_multi_body_system_mapping.active_hinge_to_joint_hinge[
                UUIDKey(active_hinge)
            ],
            np.clip(target, a_min=-active_hinge.range, a_max=active_hinge.range),
        )
