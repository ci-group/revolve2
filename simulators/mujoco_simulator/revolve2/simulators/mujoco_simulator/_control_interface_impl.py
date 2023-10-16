import mujoco

from revolve2.simulation.scene import ControlInterface, JointHinge, UUIDKey

from ._joint_hinge_ctrl_indices import JointHingeCtrlIndices


class ControlInterfaceImpl(ControlInterface):
    """Implementation of the control interface for MuJoCo."""

    _data: mujoco.MjData
    _hinge_joint_ctrl_mapping: dict[UUIDKey[JointHinge], JointHingeCtrlIndices]

    def __init__(
        self,
        data: mujoco.MjData,
        hinge_joint_ctrl_mapping: dict[UUIDKey[JointHinge], JointHingeCtrlIndices],
    ) -> None:
        """
        Initialize this object.

        :param data: The MuJoCo data to alter during control.
        :param hinge_joint_ctrl_mapping: A mapping from hinge joints to their respective indices in the MuJoCo ctrl array.
        """
        self._data = data
        self._hinge_joint_ctrl_mapping = hinge_joint_ctrl_mapping

    def set_joint_hinge_position_target(
        self, joint_hinge: JointHinge, position: float
    ) -> None:
        """
        Set the position target of a hinge joint.

        :param joint_hinge: The hinge to set the position target for.
        :param position: The position target.
        """
        data_index = self._hinge_joint_ctrl_mapping.get(UUIDKey(joint_hinge))
        assert data_index is not None, "Hinge joint does not exist in this scene."
        # Set position target
        self._data.ctrl[data_index.position] = position
        # Set velocity target
        self._data.ctrl[data_index.velocity] = 0.0
