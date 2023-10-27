from abc import ABC, abstractmethod

from ._physical_control_interface import PhysicalControlInterface
from ._physical_sensor_state import PhysicalSensorState


class PhysicalInterface(ABC):
    """Abstract implementation for interfacing with hardware."""

    @property
    @abstractmethod
    def control_interface(self) -> PhysicalControlInterface:
        """
        Get the control interface.

        :returns: The control interface.
        """

    @abstractmethod
    def read_sensor_state(self) -> PhysicalSensorState:
        """
        Read the current sensor state.

        :returns: The sensor state.
        """

    @abstractmethod
    def shutdown(self) -> None:
        """Shutdown the interface."""
