import mujoco

from revolve2.simulation.scene import ControlInterface, JointHinge, UUIDKey

from ._abstraction_to_mujoco_mapping import AbstractionToMujocoMapping


class ControlInterfaceImpl(ControlInterface):
    """Implementation of the control interface for MuJoCo."""

    _data: mujoco.MjData
    _abstraction_to_mujoco_mapping: AbstractionToMujocoMapping

    def __init__(
        self,
        data: mujoco.MjData,
        abstraction_to_mujoco_mapping: AbstractionToMujocoMapping,
    ) -> None:
        """
        Initialize this object.

        :param data: The MuJoCo data to alter during control.
        :param abstraction_to_mujoco_mapping: A mapping between simulation abstraction and mujoco.
        """
        self._data = data
        self._abstraction_to_mujoco_mapping = abstraction_to_mujoco_mapping

    def set_joint_hinge_position_target(
        self, joint_hinge: JointHinge, position: float
    ) -> None:
        """
        Set the position target of a hinge joint.

        :param joint_hinge: The hinge to set the position target for.
        :param position: The position target.
        """
        maybe_hinge_joint_mujoco = self._abstraction_to_mujoco_mapping.hinge_joint.get(
            UUIDKey(joint_hinge)
        )
        assert (
            maybe_hinge_joint_mujoco is not None
        ), "Hinge joint does not exist in this scene."
        # Set position target
        self._data.ctrl[maybe_hinge_joint_mujoco.ctrl_index_position] = position
        # Set velocity target
        self._data.ctrl[maybe_hinge_joint_mujoco.ctrl_index_velocity] = 0.0
