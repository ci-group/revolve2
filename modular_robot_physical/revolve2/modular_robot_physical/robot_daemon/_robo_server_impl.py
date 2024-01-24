import threading
import time
from typing import Any, Sequence

from pyrr import Vector3

from .._hardware_type import HardwareType
from .._protocol_version import PROTOCOL_VERSION
from ..physical_interfaces import PhysicalInterface
from ..robot_daemon_api import robot_daemon_protocol_capnp


class RoboServerImpl(robot_daemon_protocol_capnp.RoboServer.Server):  # type: ignore
    """Implements the Cap'n Proto interface."""

    _CAREFUL_STEP = 0.1

    _debug: bool
    _hardware_type: HardwareType
    _physical_interface: PhysicalInterface

    _active_pins: list[int] | None

    _enabled: bool
    _update_loop_thread: threading.Thread
    _lock: threading.Lock

    _targets: dict[int, float]  # pin -> target
    _current_targets: dict[int, float]  # pin -> target

    _measured_hinge_positions: dict[int, float]  # pin -> position
    _battery: float

    def __init__(
        self,
        debug: bool,
        hardware_type: HardwareType,
        physical_interface: PhysicalInterface,
    ) -> None:
        """
        Initialize this object.

        :param debug: Enable debug messages.
        :param hardware_type: The type of hardware this runs on.
        :param physical_interface: Interface to the actual robot.
        """
        self._debug = debug
        self._hardware_type = hardware_type
        self._physical_interface = physical_interface

        self._active_pins = None

        if self._debug:
            print("client connected")

        self._physical_interface.enable()

        self._targets = {}
        self._current_targets = {}
        self._lock = threading.Lock()

        self._measured_hinge_positions = {}
        self._battery = 0.0

    def _update_loop(self) -> None:
        assert self._active_pins is not None

        while self._enabled:
            desired_pins = []
            desired_targets = []

            # copy out of the set.
            # that is fast, setting the actual targets is slow.
            # lock for as short as possible.
            with self._lock:
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
                    match self._hardware_type:
                        case HardwareType.v1:
                            targets.append(
                                maybe_current_target
                                + min(
                                    max(
                                        (desired_target - maybe_current_target),
                                        -self._CAREFUL_STEP,
                                    ),
                                    self._CAREFUL_STEP,
                                )
                            )
                        case HardwareType.v2:  # careful mode disabled for v2. enable when running into power failures.
                            targets.append(desired_target)

            for pin, target in zip(pins, targets):
                self._current_targets[pin] = target

            self._physical_interface.set_servo_targets(pins, targets)

            # Read measured hinge positions
            if self._hardware_type is not HardwareType.v1:
                hinge_positions = self._physical_interface.get_multiple_servo_positions(
                    self._active_pins
                )

                with self._lock:
                    for pin, position in zip(self._active_pins, hinge_positions):
                        self._measured_hinge_positions[pin] = position

                battery = self._physical_interface.get_battery_level()
                with self._lock:
                    self._battery = battery

            time.sleep(1 / 60)

    def _queue_servo_targets(self, pins: list[int], targets: list[float]) -> None:
        with self._lock:
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
        args: robot_daemon_protocol_capnp.SetupArgs,
        _context: Any,
    ) -> robot_daemon_protocol_capnp.SetupResponse:
        """
        Handle a setup command.

        :param args: Arguments to the setup process.
        :returns: Whether the setup was successful.
        """
        if self._debug:
            print("setup")

        with self._lock:
            self._active_pins = [pin for pin in args.activePins]

        self._enabled = True
        self._update_loop_thread = threading.Thread(target=self._update_loop)
        self._update_loop_thread.start()

        hardware_type_str: robot_daemon_protocol_capnp.HardwareType
        match self._hardware_type:
            case HardwareType.v1:
                hardware_type_str = "v1"
            case HardwareType.v2:
                hardware_type_str = "v2"
        return robot_daemon_protocol_capnp.SetupResponse(
            versionOk=(args.version == PROTOCOL_VERSION),
            hardwareType=hardware_type_str,
        )

    async def control(
        self,
        args: robot_daemon_protocol_capnp.ControlArgsReader,
        _context: Any,
    ) -> None:
        """
        Handle control commands.

        :param args: Args to the function.
        """
        if self._debug:
            print("control")

        self._queue_servo_targets(
            [pin_control.pin for pin_control in args.setPins],
            [pin_control.target for pin_control in args.setPins],
        )

    async def readSensors(
        self,
        args: robot_daemon_protocol_capnp.ReadSensorsArgsReader,
        _context: Any,
    ) -> robot_daemon_protocol_capnp.SensorReadings:
        """
        Handle readSensors.

        Stub that currently does reads nothing.

        :param args: Args to the function.
        :returns: The readings.
        """
        if self._debug:
            print("read_sensors")

        return self._get_sensor_readings(args.readPins)

    async def controlAndReadSensors(
        self,
        args: robot_daemon_protocol_capnp.ControlAndReadSensorsArgsReader,
        _context: Any,
    ) -> robot_daemon_protocol_capnp.SensorReadings:
        """
        Handle controlAndReadSensors.

        Currently reads nothing.

        :param args: Args to the function.
        :returns: The readings.
        """
        if self._debug:
            print("control_and_read_sensors")

        self._queue_servo_targets(
            [pin_control.pin for pin_control in args.setPins],
            [pin_control.target for pin_control in args.setPins],
        )

        return self._get_sensor_readings(args.readPins)

    def _get_sensor_readings(
        self, pins: Sequence[int]
    ) -> robot_daemon_protocol_capnp.SensorReadings:
        if self._active_pins is None:
            raise RuntimeError("setup must be called first.")

        pins_readings: list[float] = []
        with self._lock:
            for pin in pins:
                if pin not in self._active_pins:
                    raise ValueError(
                        "Position requested for pin that was not flagged as active during setup."
                    )
                value = self._measured_hinge_positions.get(pin)
                if value is None:
                    value = 0.0
                pins_readings.append(value)

            battery = self._physical_interface.get_battery_level()

            imu_orientation = self._physical_interface.get_imu_orientation()
            imu_specific_force = self._physical_interface.get_imu_specific_force()
            imu_angular_rate = self._physical_interface.get_imu_angular_rate()

        return robot_daemon_protocol_capnp.SensorReadings(
            pins=pins_readings,
            battery=battery,
            imuOrientation=self._vector3_to_capnp(imu_orientation),
            imuSpecificForce=self._vector3_to_capnp(imu_specific_force),
            imuAngularRate=self._vector3_to_capnp(imu_angular_rate),
        )

    @staticmethod
    def _vector3_to_capnp(vector: Vector3) -> Vector3:
        return robot_daemon_protocol_capnp.Vector3(
            x=float(vector.x), y=float(vector.y), z=float(vector.z)
        )
