from pyrr import Quaternion

from revolve2.modular_robot.sensor_state import IMUSensorState
from revolve2.simulation.scene import MultiBodySystem, SimulationState


class IMUSensorStateImpl(IMUSensorState):
    """Implements the  IMU sensor state."""

    _simulation_state: SimulationState
    _multi_body_system: MultiBodySystem

    def __init__(
        self, simulation_state: SimulationState, multi_body_system: MultiBodySystem
    ) -> None:
        """
        Initialize this object.

        :param simulation_state: The state of the simulation.
        :param multi_body_system: The multi body system this imu is attached to.
        """
        self._simulation_state = simulation_state
        self._multi_body_system = multi_body_system

    @property
    def specific_force(self) -> float:
        """
        Get the measured specific force.

        :returns: The measured specific force.
        """
        return 0

    @property
    def angular_rate(self) -> float:
        """
        Get the measured angular rate.

        :returns: The measured angular rate.
        """
        return 0

    @property
    def orientation(self) -> Quaternion:
        """
        Get the measured position of the active hinge.

        :returns: The measured position.
        """
        return self._simulation_state.get_multi_body_system_pose(
            self._multi_body_system
        ).orientation
