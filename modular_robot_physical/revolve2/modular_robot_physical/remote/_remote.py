import socket
import time
from typing import Any

import capnp
import paramiko
import paramiko.channel

from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain import BrainInstance

from .._config import Config
from .._protocol_version import PROTOCOL_VERSION
from .._standard_port import STANDARD_PORT
from .._uuid_key import UUIDKey
from ..robot_daemon_api import robot_daemon_protocol_capnp
from ._modular_robot_control_interface_impl import ModularRobotControlInterfaceImpl
from ._modular_robot_sensor_state_impl import ModularRobotSensorStateImpl


class Remote:
    """Remote control for physical modular robot that locally executes a brain and transmits it over ssh using `stream_brain`."""

    _INITIAL_SETUP_DELAY = 0.5

    _config: Config
    _debug: bool

    _stream_brain_ssh_channel: paramiko.Channel
    _stream_socket: socket.socket

    _controller: BrainInstance

    _client: capnp.TwoPartyClient
    _service: Any  # The Cap'n Proto interface

    def __init__(
        self,
        config: Config,
        hostname: str,
        port: int = STANDARD_PORT,
        debug: bool = False,
    ) -> None:
        """
        Initialize this object.

        :param hostname: Network hostname.
        :param port: The port to use for the robots stream service.
        :param config: The modular robot configuration.
        :param debug: Whether to print debug information.
        :raises ConnectionRefusedError: When a connection cannot be made to the robot.
        """
        self._config = config
        self._debug = debug

        self._controller = self._config.modular_robot.brain.make_instance()

        try:
            self._client = capnp.TwoPartyClient(f"{hostname}:{port}")
            self._service = self._client.bootstrap().cast_as(
                robot_daemon_protocol_capnp.RoboServer
            )
        except ConnectionRefusedError:
            raise ConnectionRefusedError("Could not connect to robot.")

    def _set_active_hinge_targets(
        self, active_hinges_and_targets: list[tuple[UUIDKey[ActiveHinge], float]]
    ) -> None:
        """
        Set the target for active hinges.

        :param active_hinges_and_targets: The active hinges and their targets.
        """
        pins = [
            self._config.hinge_mapping[active_hinge]
            for active_hinge, _ in active_hinges_and_targets
        ]
        inverses = [self._config.inverse_servos.get(pin, False) for pin in pins]
        targets = [
            (-1.0 if inverse else 1.0)
            * min(max(target, -active_hinge.value.range), active_hinge.value.range)
            for (active_hinge, target), inverse in zip(
                active_hinges_and_targets, inverses
            )
        ]

        self._service.control(
            robot_daemon_protocol_capnp.ControlCommands(
                pins=[
                    robot_daemon_protocol_capnp.PinControl(pin=pin, target=target)
                    for pin, target in zip(pins, targets)
                ]
            )
        ).wait()

    def _active_hinge_targets_to_pin_controls(
        self, active_hinges_and_targets: list[tuple[UUIDKey[ActiveHinge], float]]
    ) -> list[robot_daemon_protocol_capnp.PinControl]:
        pins = [
            self._config.hinge_mapping[active_hinge]
            for active_hinge, _ in active_hinges_and_targets
        ]
        inverses = [self._config.inverse_servos.get(pin, False) for pin in pins]
        targets = [
            (-1.0 if inverse else 1.0)
            * min(max(target, -active_hinge.value.range), active_hinge.value.range)
            for (active_hinge, target), inverse in zip(
                active_hinges_and_targets, inverses
            )
        ]
        return [
            robot_daemon_protocol_capnp.PinControl(pin=pin, target=target)
            for pin, target in zip(pins, targets)
        ]

    def prepare(self) -> None:
        """
        Prepare the robot for running the brain.

        :raises RuntimeError: If setup was not successful.
        """
        # Setup the robot and check protocol version
        setup_response: robot_daemon_protocol_capnp.SetupResponse = (
            self._service.setup(
                robot_daemon_protocol_capnp.Setupargs(version=PROTOCOL_VERSION)
            )
            .wait()
            .response
        )
        if not setup_response.versionOk:
            raise RuntimeError("Protocol version does not match for robot.")

        # Set hinges to initial positions.
        pin_controls = self._active_hinge_targets_to_pin_controls(
            [
                (active_hinge, self._config.initial_hinge_positions[active_hinge])
                for active_hinge in self._config.hinge_mapping
            ]
        )
        self._service.control(
            robot_daemon_protocol_capnp.ControlCommands(pins=pin_controls)
        ).wait()

    def run(self) -> None:
        """Run the brain and remote."""
        control_period = 1 / self._config.control_frequency

        start_time = time.time()
        last_update_time = start_time

        while (current_time := time.time()) - start_time < self._config.run_duration:
            # Sleep until next control update
            next_update_at = last_update_time + control_period
            if current_time < next_update_at:
                time.sleep(next_update_at - current_time)
                last_update_time = next_update_at
                elapsed_time = control_period
            else:
                print(
                    f"WARNING: Loop is lagging {next_update_at - current_time} seconds behind the set update frequency. Is your control function too slow?"
                )
                elapsed_time = last_update_time - current_time
                last_update_time = current_time

            # Get targets from brain
            control_interface = ModularRobotControlInterfaceImpl()
            sensor_state = ModularRobotSensorStateImpl()
            self._controller.control(
                elapsed_time,
                sensor_state=sensor_state,
                control_interface=control_interface,
            )

            # Reading sensors will come in a later update.
            pin_controls = self._active_hinge_targets_to_pin_controls(
                control_interface._set_active_hinges
            )
            self._service.control(
                robot_daemon_protocol_capnp.ControlCommands(pins=pin_controls)
            ).wait()
