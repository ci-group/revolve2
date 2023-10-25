from revolve2.modular_robot.body.base import ActiveHinge

from .._physical_control_interface import PhysicalControlInterface
from .._physical_interface import PhysicalInterface
from .._physical_sensor_state import PhysicalSensorState
from ._v1_physical_control_interface import V1PhysicalControlInterface
from ._v1_physical_sensor_state import V1PhysicalSensorState


class V1PhysicalInterface(PhysicalInterface):
    """Implements PhysicalSensorStateReader for v1 hardware."""

    _control_interface: V1PhysicalControlInterface

    def __init__(
        self,
        debug: bool,
        dry: bool,
        hinge_mapping: dict[ActiveHinge, int],
        inverse_pin: dict[int, bool],
    ) -> None:
        """
        Initialize this object..

        :param debug: If debugging messages are activated.
        :param dry: If dry.
        :param hinge_mapping: The modular robots hinges mapped to servos of the physical robot.
        :param inverse_pin: If pins are inversed.
        """
        self._control_interface = V1PhysicalControlInterface(
            debug=debug, dry=dry, hinge_mapping=hinge_mapping, inverse_pin=inverse_pin
        )

    @property
    def control_interface(self) -> PhysicalControlInterface:
        """
        Get the control interface.

        :returns: The control interface.
        """
        return self._control_interface

    def read_sensor_state(self) -> PhysicalSensorState:
        """
        Read the current sensor state.

        :returns: The sensor state.
        """
        return V1PhysicalSensorState()

    def shutdown(self) -> None:
        """Shutdown the interface."""
        self._control_interface.stop_pwm()
