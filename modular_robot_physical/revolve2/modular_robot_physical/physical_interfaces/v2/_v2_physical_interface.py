from revolve2.modular_robot.body.base import ActiveHinge

from ..._uuid_key import UUIDKey
from .._physical_control_interface import PhysicalControlInterface
from .._physical_interface import PhysicalInterface
from .._physical_sensor_state import PhysicalSensorState
from ._v2_physical_control_interface import V2PhysicalControlInterface
from ._v2_physical_sensor_state import V2PhysicalSensorState


class V2PhysicalInterface(PhysicalInterface):
    """Implements PhysicalSensorStateReader for v2 hardware."""

    _control_interface: V2PhysicalControlInterface

    def __init__(
        self,
        debug: bool,
        dry: bool,
        hinge_mapping: dict[UUIDKey[ActiveHinge], int],
        inverse_pin: dict[int, bool],
    ) -> None:
        """
        Initialize this object..

        :param debug: If debugging messages are activated.
        :param dry: If dry.
        :param hinge_mapping: The modular robots hinges mapped to servos of the physical robot.
        :param inverse_pin: If pins are inversed.
        """
        self._control_interface = V2PhysicalControlInterface(
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
        return V2PhysicalSensorState()

    def shutdown(self) -> None:
        """Shutdown the interface."""
        self._control_interface.shutdown()
