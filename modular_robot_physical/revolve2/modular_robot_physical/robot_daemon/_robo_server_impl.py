from typing import Any

from .._protocol_version import PROTOCOL_VERSION
from ..physical_interfaces import PhysicalInterface
from ..robot_daemon_api import robot_daemon_protocol_capnp


class RoboServerImpl(robot_daemon_protocol_capnp.RoboServer.Server):  # type: ignore
    """Implements the Cap'n Proto interface."""

    _debug: bool
    _physical_interface: PhysicalInterface

    def __init__(self, debug: bool, physical_interface: PhysicalInterface) -> None:
        """
        Initialize this object.

        :param debug: Enable debug messages.
        :param physical_interface: Interface to the actual robot.
        """
        self._debug = debug
        self._physical_interface = physical_interface

        if self._debug:
            print("client connected")

        self._physical_interface.enable()

    def cleanup(self) -> None:
        """Stop the server and sets everything to low power."""
        if self._debug:
            print("client disconnected.")
        self._physical_interface.disable()

    def setup(
        self,
        setupargs: robot_daemon_protocol_capnp.Setupargs,
        _context: Any,
    ) -> robot_daemon_protocol_capnp.SetupResponse:
        """
        Handle a setup command.

        :param setupargs: Arguments to the setup process.
        :returns: Whether the setup was successful.
        """
        if self._debug:
            print("setup")

        return robot_daemon_protocol_capnp.SetupResponse(
            versionOk=(setupargs.version == PROTOCOL_VERSION)
        )

    def control(
        self,
        commands: robot_daemon_protocol_capnp.ControlCommandsReader,
        _context: Any,
    ) -> None:
        """
        Handle control commands.

        :param commands: The commands to perform.
        """
        if self._debug:
            print("control")

        for pin_control in commands.pins:
            self._physical_interface.set_servo_target(
                pin=pin_control.pin, target=pin_control.target
            )

    def readSensors(
        self,
        _context: Any,
    ) -> robot_daemon_protocol_capnp.SensorReadings:
        """
        Handle readSensors.

        Stub that currently does reads nothing.

        :returns: The readings.
        """
        if self._debug:
            print("read_sensors")

        return robot_daemon_protocol_capnp.SensorReadings()

    def controlAndReadSensors(
        self,
        commands: robot_daemon_protocol_capnp.ControlCommandsReader,
        _context: Any,
    ) -> robot_daemon_protocol_capnp.SensorReadings:
        """
        Handle controlAndReadSensors.

        Currently reads nothing.

        :param commands: The commands to perform.
        :returns: The readings.
        """
        if self._debug:
            print("control_and_read_sensors")

        for pin_control in commands.pins:
            self._physical_interface.set_servo_target(
                pin=pin_control.pin, target=pin_control.target
            )

        return robot_daemon_protocol_capnp.SensorReadings()
