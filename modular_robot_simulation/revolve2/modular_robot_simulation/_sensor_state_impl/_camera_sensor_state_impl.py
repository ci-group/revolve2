import numpy as np
from numpy.typing import NDArray
from revolve2.modular_robot.sensor_state import CameraSensorState
from revolve2.simulation.scene import MultiBodySystem, SimulationState
from revolve2.simulation.scene.sensors import CameraSensor


class CameraSensorStateImpl(CameraSensorState):
    """The simulation implementation of the camera sensor state."""

    _simulation_state: SimulationState
    _multi_body_system: MultiBodySystem
    _camera: CameraSensor

    def __init__(
        self,
        simulation_state: SimulationState,
        multi_body_system: MultiBodySystem,
        camera: CameraSensor,
    ) -> None:
        """
        Initialize this object.

        :param simulation_state: The state of the simulation.
        :param multi_body_system: The multi body system this imu is attached to.
        :param camera: The camera sensor.
        """
        self._simulation_state = simulation_state
        self._multi_body_system = multi_body_system
        self._camera = camera

    @property
    def image(self) -> NDArray[np.uint8]:
        """
        Get the current image.

        :returns: The image.
        """
        return self._simulation_state.get_camera_view(self._camera)
