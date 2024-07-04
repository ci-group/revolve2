from pyrr import Quaternion, Vector3
from revolve2.modular_robot.sensor_state import IMUSensorState
from revolve2.simulation.scene import MultiBodySystem, SimulationState
from revolve2.simulation.scene.sensors import IMUSensor


class IMUSensorStateImpl(IMUSensorState):
    """Implements the  IMU sensor state."""

    _simulation_state: SimulationState
    _multi_body_system: MultiBodySystem
    _core_imu: IMUSensor

    def __init__(
        self,
        simulation_state: SimulationState,
        multi_body_system: MultiBodySystem,
        imu: IMUSensor,
    ) -> None:
        """
        Initialize this object.

        :param simulation_state: The state of the simulation.
        :param multi_body_system: The multi body system this imu is attached to.
        :param imu: The imu attached to the core.
        """
        self._simulation_state = simulation_state
        self._multi_body_system = multi_body_system
        self._imu = imu

    @property
    def specific_force(self) -> Vector3:
        """
        Get the measured specific force.

        :returns: The measured specific force.
        """
        return self._simulation_state.get_imu_specific_force(self._imu)

    @property
    def angular_rate(self) -> Vector3:
        """
        Get the measured angular rate.

        :returns: The measured angular rate.
        """
        return self._simulation_state.get_imu_angular_rate(self._imu)

    @property
    def orientation(self) -> Quaternion:
        """
        Get the measured orientation.

        :returns: The measured position.
        """
        return self._simulation_state.get_multi_body_system_pose(
            self._multi_body_system
        ).orientation
