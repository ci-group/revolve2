import threading
import time
from typing import Any

from .._protocol_version import PROTOCOL_VERSION
from ..physical_interfaces import PhysicalInterface
from ..robot_daemon_api import robot_daemon_protocol_capnp


class RoboServerImpl(robot_daemon_protocol_capnp.RoboServer.Server):  # type: ignore
    """Implements the Cap'n Proto interface."""

    _CAREFUL_STEP = 0.1

    _debug: bool
    _physical_interface: PhysicalInterface

    _enabled: bool
    _update_loop_thread: threading.Thread
    _targets: dict[int, float]  # pin -> target
    _current_targets: dict[int, float]  # pin -> target
    _targets_lock: threading.Lock

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

        self._targets = {}
        self._current_targets = {}
        self._targets_lock = threading.Lock()

        self._enabled = True
        self._update_loop_thread = threading.Thread(target=self._update_loop)
        self._update_loop_thread.start()

    def _update_loop(self) -> None:
        while self._enabled:
            desired_pins = []
            desired_targets = []

            # copy out of the set.
            # that is fast, setting the actual targets is slow.
            # lock for as short as possible.
            with self._targets_lock:
                for pin, target in self._targets.items():
                    desired_pins.append(pin)
                    desired_targets.append(target)

            pins: list[int] = []
            targets: list[float] = []
            for desired_pin, desired_target in zip(desired_pins, desired_targets):
                pins.append(desired_pin)

                maybe_current_target = self._current_targets.get(desired_pin)
                if maybe_current_target is None:
                    targets.append(desired_target)
                else:
                    targets.append(
                        min(
                            max(
                                (desired_target - maybe_current_target),
                                -self._CAREFUL_STEP,
                            ),
                            self._CAREFUL_STEP,
                        )
                    )
            for pin, target in zip(pins, targets):
                self._current_targets[pin] = target

            self._physical_interface.set_servo_targets(pins, targets)

            time.sleep(1 / 60)

    def _queue_servo_targets(self, pins: list[int], targets: list[float]) -> None:
        with self._targets_lock:
            for pin, target in zip(pins, targets):
                self._targets[pin] = target

    def cleanup(self) -> None:
        """Stop the server and sets everything to low power."""
        if self._debug:
            print("client disconnected.")

        if self._debug:
            print("stopping background thread.")
        self._enabled = False
        self._update_loop_thread.join()

        self._physical_interface.disable()

    async def setup(
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

    async def control(
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

        self._queue_servo_targets(
            [pin_control.pin for pin_control in commands.pins],
            [pin_control.target for pin_control in commands.pins],
        )

    async def readSensors(
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

        self._queue_servo_targets(
            [pin_control.pin for pin_control in commands.pins],
            [pin_control.target for pin_control in commands.pins],
        )

        return robot_daemon_protocol_capnp.SensorReadings()
