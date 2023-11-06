import json
import time

import paramiko
import paramiko.channel

from .._config import Config
from ..physical_interfaces import HardwareType
from .._uuid_key import UUIDKey
from ._modular_robot_control_interface_impl import ModularRobotControlInterfaceImpl
from ._modular_robot_sensor_state_impl import ModularRobotSensorStateImpl
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain import BrainInstance
import socket


class Remote:
    """Remote control for physical modular robot that locally executes a brain and transmits it over ssh using `stream_brain`."""

    _INITIAL_SETUP_DELAY = 0.5
    _STREAM_PORT = 20812

    _config: Config
    _stream_brain_ssh_channel: paramiko.Channel
    _stream_socket: socket.socket

    _debug: bool

    _controller: BrainInstance

    def __init__(
        self,
        hostname: str,
        username: str,
        password: str,
        hardware_type: HardwareType,
        config: Config,
        debug: bool = False,
    ) -> None:
        """
        Initialize this object.

        :param hostname: Network hostname.
        :param username: SSH username.
        :param password: SSH password.
        :param hardware_type: The type of hardware of the physical modular robot.
        :param config: The modular robot configuration.
        :param debug: Whether to print debug information.
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
            )
        except (paramiko.AuthenticationException, TimeoutError) as e:
            raise ConnectionRefusedError(
                f"Could not connect to the robot: {e}"
            ) from None

        try:
            self._open_stream_socket(hostname=hostname)
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
    ) -> None:
        """
        Start the brain stream service.

        :param hostname: The hostname.
        :param username: SSH username.
        :param password: SSH password.
        :hardware_type: The type of hardware.
        """

        # Connect to the robot
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(hostname, username=username, password=password, timeout=3.0)

        transport = ssh.get_transport()
        self._stream_brain_ssh_channel = transport.open_session()

        # Start the stream brain service.
        actual_command = f"stream_brain --port {self._STREAM_PORT} --hardware {hardware_type.name} --pins {' '.join([str(pin) for pin in self._config.hinge_mapping.values()])}"
        self._stream_brain_ssh_channel.exec_command(f'bash -l -c "{actual_command}"')

        # Wait until the service starts.
        time.sleep(1.0)

        # Check if the service successfully started.
        if self._stream_brain_ssh_channel.exit_status_ready():
            exit_status = self._stream_brain_ssh_channel.recv_exit_status()
            if exit_status != 0:
                # The server exited with an error, attempt to read the error stream.
                error_msg = self._stream_brain_ssh_channel.recv_stderr(1024).decode()
                raise RuntimeError(
                    f"Brain stream service on the robot exited with error status {exit_status}: {error_msg}"
                )
            else:
                # The server exited cleanly but unexpectedly.
                raise RuntimeError(
                    "Brain stream service on the robot exited unexpectedly but reported no error."
                )

    def _open_stream_socket(self, hostname: str) -> None:
        self._stream_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._stream_socket.connect((hostname, self._STREAM_PORT))

    def _send_command(self, command: bytes):
        self._stream_socket.sendall(command)

    def set_active_hinge_targets(
        self, active_hinges_and_targets: list[tuple[UUIDKey[ActiveHinge], float]]
    ) -> None:
        """
        Set the target for active hinges.

        :param active_hinges_and_targets: The active hinges and their targets.
        :raises RuntimeError: In case of unexpected client response.
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
            self._send_command(cmd_str.encode())
        except BrokenPipeError as e:
            raise BrokenPipeError(f"Lost connection to the robot: {e}") from None

    def _set_active_hinges_initial_positions(self) -> None:
        """Set all servos to their initial positions."""
        for active_hinge in self._config.hinge_mapping:
            # Set active hinge to initial position.
            self.set_active_hinge_targets(
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

            self.set_active_hinge_targets(control_interface._set_active_hinges)
