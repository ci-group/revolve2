from pyrr import Quaternion

from revolve2.modular_robot.sensor_state import IMUSensorState
from revolve2.simulation.scene import MultiBodySystem, SimulationState, RigidBody


class IMUSensorStateImpl(IMUSensorState):
    """Implements the  IMU sensor state."""

    _simulation_state: SimulationState
    _multi_body_system: MultiBodySystem
    _rigid_body: RigidBody

    def __init__(
        self,
        simulation_state: SimulationState,
        multi_body_system: MultiBodySystem,
        rigid_body: RigidBody,
    ) -> None:
        """
        Initialize this object.

        :param simulation_state: The state of the simulation.
        :param multi_body_system: The multi body system this imu is attached to.
        :param rigid_body: The rigid body the imu is attached to.
        """
        self._simulation_state = simulation_state
        self._multi_body_system = multi_body_system
        self._rigid_body = rigid_body

    @property
    def specific_force(self) -> float:
        """
        Get the measured specific force.

        :returns: The measured specific force.
        """
        return self._simulation_state.get_rigid_body_imu_specific_force(
            self._rigid_body
        )

    @property
    def angular_rate(self) -> float:
        """
        Get the measured angular rate.

        :returns: The measured angular rate.
        """
        return self._simulation_state.get_rigid_body_imu_angular_rate(self._rigid_body)

    @property
    def orientation(self) -> Quaternion:
        """
        Get the measured orientation.

        :returns: The measured position.
        """
        return self._simulation_state.get_multi_body_system_pose(
            self._multi_body_system
        ).orientation
