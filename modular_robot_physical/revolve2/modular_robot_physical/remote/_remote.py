import json
import socket
import struct
import time

import paramiko
import paramiko.channel

from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain import BrainInstance

from .._config import Config
from .._protocol_version import PROTOCOL_VERSION
from .._uuid_key import UUIDKey
from ..physical_interfaces import HardwareType
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

    def __init__(
        self,
        hostname: str,
        username: str,
        password: str,
        hardware_type: HardwareType,
        config: Config,
        debug: bool = False,
        port: int = 20812,
    ) -> None:
        """
        Initialize this object.

        :param hostname: Network hostname.
        :param username: SSH username.
        :param password: SSH password.
        :param hardware_type: The type of hardware of the physical modular robot.
        :param config: The modular robot configuration.
        :param debug: Whether to print debug information.
        :param port: The port to use for the robots stream service.
        :raises ConnectionRefusedError: When a connection cannot be made to the robot.
        """
        self._config = config
        self._debug = debug

        self._controller = self._config.modular_robot.brain.make_instance()

        try:
            self._start_stream_brain(
                hostname=hostname,
                username=username,
                password=password,
                hardware_type=hardware_type,
                port=port,
            )
        except (paramiko.AuthenticationException, TimeoutError) as e:
            raise ConnectionRefusedError(
                f"Could not connect to the robot: {e}"
            ) from None

        try:
            self._open_stream_socket(hostname=hostname, port=port)
        except ConnectionRefusedError as e:
            raise ConnectionRefusedError(
                f"Could not connect to the robot: {e}"
            ) from None

    def _start_stream_brain(
        self,
        hostname: str,
        username: str,
        password: str,
        hardware_type: HardwareType,
        port: int,
    ) -> None:
        """
        Start the brain stream service.

        :param hostname: The hostname.
        :param username: SSH username.
        :param password: SSH password.
        :param hardware_type: The type of hardware.
        :param port: The port to use for the stream service.
        :raises RuntimeError: If the stream service could not be started.
        """
        # Connect to the robot
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(hostname, username=username, password=password, timeout=3.0)

        transport = ssh.get_transport()
        assert transport is not None
        self._stream_brain_ssh_channel = transport.open_session()
        self._stream_brain_ssh_channel.get_pty()

        # Start the stream brain service.
        actual_command = f"stream_brain --required-version {PROTOCOL_VERSION} --port {port} --hardware {hardware_type.name} --pins {' '.join([str(pin) for pin in self._config.hinge_mapping.values()])}"
        self._stream_brain_ssh_channel.exec_command(f'bash -l -c "{actual_command}"')

        # Wait for the service to start.
        start_time = time.time()
        while True:
            if time.time() - start_time > 5:
                raise RuntimeError("Brain stream service did not start within timeout.")

            # Check if there was an error.
            if self._stream_brain_ssh_channel.exit_status_ready():
                exit_status = self._stream_brain_ssh_channel.recv_exit_status()
                if exit_status != 0:
                    # The service exited with an error, attempt to read the error stream.
                    error_msg = self._stream_brain_ssh_channel.recv_stderr(
                        1024
                    ).decode()
                    raise RuntimeError(
                        f"Brain stream service on the robot exited with error status {exit_status}: {error_msg}"
                    )
                else:
                    # The service exited cleanly but unexpectedly.
                    raise RuntimeError(
                        "Brain stream service on the robot exited unexpectedly but reported no error."
                    )

            # Check if the service started.
            if self._stream_brain_ssh_channel.recv_ready():
                output = self._stream_brain_ssh_channel.recv(1024).decode("utf-8")
                if "///start up complete\\\\\\" in output:
                    break

    def _open_stream_socket(self, hostname: str, port: int) -> None:
        self._stream_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._stream_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        self._stream_socket.connect((hostname, port))

    def _send_command(self, command: str) -> None:
        """
        Send a command.

        :param command: The command to send.
        """
        self._stream_socket.sendall(struct.pack(">I", len(command)) + command.encode())

    def _set_active_hinge_targets(
        self, active_hinges_and_targets: list[tuple[UUIDKey[ActiveHinge], float]]
    ) -> None:
        """
        Set the target for active hinges.

        :param active_hinges_and_targets: The active hinges and their targets.
        :raises BrokenPipeError: In case the connection closed.
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

        # Set targets on robot
        cmd = {
            "cmd": "setpins",
            "pins": [
                {
                    "pin": pin,
                    "target": target,
                }
                for pin, target in zip(pins, targets)
            ],
        }
        cmd_str = json.dumps(cmd)

        if self._debug:
            print(f"Sending command: {cmd_str}")

        try:
            self._send_command(cmd_str)
        except (ConnectionResetError, BrokenPipeError) as e:
            raise BrokenPipeError(f"Lost connection to the robot: {e}") from None

    def _set_active_hinges_initial_positions(self) -> None:
        """Set all servos to their initial positions."""
        for active_hinge in self._config.hinge_mapping:
            # Set active hinge to initial position.
            self._set_active_hinge_targets(
                [(active_hinge, self._config.initial_hinge_positions[active_hinge])]
            )

            # Wait a bit until setting the next active hinge.
            time.sleep(self._INITIAL_SETUP_DELAY)

    def prepare(self) -> None:
        """Prepare the robot for running the brain."""
        self._set_active_hinges_initial_positions()

    def run(self) -> None:
        """Run the brain and remote."""
        control_period = 1 / self._config.control_frequency

        start_time = time.time()
        last_update_time = start_time

        while (current_time := time.time()) - start_time < self._config.run_duration:
            # Sleep until next control update
            time.sleep(control_period)

            elapsed_time = current_time - last_update_time
            last_update_time = current_time

            # Get targets from brain
            control_interface = ModularRobotControlInterfaceImpl()
            sensor_state = ModularRobotSensorStateImpl()
            self._controller.control(
                elapsed_time,
                sensor_state=sensor_state,
                control_interface=control_interface,
            )

            self._set_active_hinge_targets(control_interface._set_active_hinges)
